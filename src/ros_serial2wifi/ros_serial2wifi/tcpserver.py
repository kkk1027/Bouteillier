#!/usr/bin/env python3
"""TCP ↔ pty 串口桥接服务端。

雷达转接板作为 TCP 客户端连入本节点，本节点把 TCP 字节流透明转发到
/tmp/tty_laser（pty），供 ydlidar 驱动当作真实串口读取。

2026-10-30 修复（此前导致 /scan 频繁 "Check Sum 不符" 与丢帧）：
  1. 原实现 `os.write(write_fd, data)` 忽略返回值。pty/socket 都可能
     **部分写入**，剩余字节被静默丢弃，直接造成雷达帧校验失败。
     现在改为循环写直到写完，写不完的字节留到下次 POLLOUT。
  2. 原实现每次只 `os.read(fd, 256)`，吞吐受限；改为可配置的 4096 字节。
  3. 原实现 5 秒无数据就主动断开。雷达/驱动任一方向短暂静默都会触发，
     造成反复掉线重连。现在默认不断开（idle_disconnect_sec=0），仅告警。
  4. 收发改为非阻塞 + 双向缓冲，避免某一方向阻塞（如 ydlidar 读得慢导致
     pty 缓冲满）拖死整个转发循环，进而让雷达侧 TCP 发送阻塞、丢数据。
"""

import os
import pty
import select
import socket
import time
import tty

import rclpy
from rclpy.node import Node


class TcpSocketServerNode(Node):
    def __init__(self):
        super().__init__('tcp_socket_server_node')

        # ---------- ROS 2 参数 ----------
        self.declare_parameter('tcp_port', 8889)
        self.declare_parameter('serial_port', '/tmp/laserport')
        # 单次读取上限；越大吞吐越高，单次时延略增
        self.declare_parameter('read_chunk', 4096)
        # 雷达侧多久没有字节就判定「转接板假死」并主动断开，逼它重连。
        #
        # 实测（2026-10-30）：热点拥塞时，雷达转接板会保持 TCP ESTABLISHED
        # 但彻底停止转发串口字节，本端 Send-Q 里积着几十字节永远发不出去。
        # 此时不主动断连就永远恢复不了（上半连接已死，内核看不出）。
        # 雷达正常时持续 5Hz 出数据，所以 5s 无字节必然是异常。
        # 设 0 = 关闭该检测（不推荐）。
        self.declare_parameter('idle_disconnect_sec', 5.0)
        # 写超时（毫秒）：对端不 ACK 时让写操作快速失败，而不是无限阻塞
        self.declare_parameter('tcp_user_timeout_ms', 15000)
        # 单向积压上限：下游（pty 或 socket）读得慢时，旧字节会堆在缓冲里。
        # 雷达是流式数据，积压的旧字节已经没用，超过上限就丢最旧的，
        # 避免内存无限增长与延迟越排越大。
        self.declare_parameter('max_buffer_bytes', 262144)
        # 统计日志周期；0 = 不打印
        self.declare_parameter('stats_period_sec', 30.0)

        self.tcp_port = int(
            self.get_parameter('tcp_port').get_parameter_value().integer_value)
        self.serial_port = str(
            self.get_parameter('serial_port').get_parameter_value().string_value)
        self.read_chunk = max(256, int(self.get_parameter('read_chunk').value))
        self.idle_disconnect_sec = float(
            self.get_parameter('idle_disconnect_sec').value)
        self.stats_period_sec = float(
            self.get_parameter('stats_period_sec').value)
        self.tcp_user_timeout_ms = int(
            self.get_parameter('tcp_user_timeout_ms').value)
        self.max_buffer_bytes = max(
            4096, int(self.get_parameter('max_buffer_bytes').value))

        # ---------- 统计量 ----------
        self.stat_to_master = 0
        self.stat_to_client = 0
        self.stat_dropped = 0
        self.stat_sessions = 0
        self._last_stats_time = time.time()

    # ------------------------------------------------------------------
    @staticmethod
    def _write_nonblocking(fd, buf):
        """非阻塞写：写多少算多少，返回尚未写出的剩余字节。"""
        view = memoryview(buf)
        total = 0
        while total < len(buf):
            try:
                written = os.write(fd, view[total:])
            except BlockingIOError:
                break
            except InterruptedError:
                continue
            if written <= 0:
                break
            total += written
        return bytes(view[total:])

    @staticmethod
    def _update_mask(poller, fd, out_buf):
        """无待发数据时只监听 POLLIN，避免 poll 空转占满 CPU。"""
        events = select.POLLIN
        if out_buf:
            events |= select.POLLOUT
        try:
            poller.modify(fd, events)
        except Exception:  # noqa: BLE001 - fd 可能已注销
            pass

    def _flush(self, master, client_fd, to_master, to_client):
        """把两个方向的缓冲尽量写出。

        注意：`_write_nonblocking` 没写出去的部分会留在缓冲里下次再写，
        **不算丢弃**。只有缓冲超过 max_buffer_bytes 而不得不丢掉最旧的
        字节时，才计入 stat_dropped。
        """
        if to_master:
            self._drain(master, to_master)
        if to_client:
            self._drain(client_fd, to_client)

    def _drain(self, fd, buf):
        left = self._write_nonblocking(fd, bytes(buf))
        del buf[:]
        buf.extend(left)
        if len(buf) > self.max_buffer_bytes:
            overflow = len(buf) - (self.max_buffer_bytes // 2)
            del buf[:overflow]
            self.stat_dropped += overflow

    def _maybe_log_stats(self, force=False):
        if self.stats_period_sec <= 0:
            return
        now = time.time()
        if not force and now - self._last_stats_time < self.stats_period_sec:
            return
        self._last_stats_time = now
        self.get_logger().info(
            f'桥接统计: 雷达→驱动 {self.stat_to_master} 字节, '
            f'驱动→雷达 {self.stat_to_client} 字节, '
            f'积压溢出丢弃 {self.stat_dropped} 字节, 会话数 {self.stat_sessions}')

    # ------------------------------------------------------------------
    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.tcp_port))
        server.listen(1)

        master, slave = pty.openpty()
        # 安全网：把 pty 置为 raw，避免行规程翻译（ICRNL/OPOST/IXON）破坏
        # 雷达的二进制协议。ydlidar 打开端口后自己也会 cfmakeraw，此处
        # 只是保证「驱动打开之前」流入的数据也不会被改写。
        try:
            tty.setraw(master)
        except Exception as exc:  # noqa: BLE001 - 非致命
            self.get_logger().warn(f'设置 pty raw 模式失败: {exc}')

        if os.path.lexists(self.serial_port):
            os.remove(self.serial_port)
        os.symlink(os.ttyname(slave), self.serial_port)

        self.get_logger().info(
            f'TCP端口:{self.tcp_port}，已映射到串口设备:{self.serial_port} '
            f'(read_chunk={self.read_chunk}, '
            f'idle_disconnect_sec={self.idle_disconnect_sec})')

        os.set_blocking(master, False)
        poller = select.poll()

        try:
            while rclpy.ok():
                self.get_logger().info('等待接受连接..')
                server.settimeout(None)
                client, client_address = server.accept()

                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 18)
                client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 18)
                # 对端不 ACK 时尽快报错（热点拥塞时避免写操作长期阻塞）
                if self.tcp_user_timeout_ms > 0 and hasattr(
                        socket, 'TCP_USER_TIMEOUT'):
                    try:
                        client.setsockopt(socket.IPPROTO_TCP,
                                          socket.TCP_USER_TIMEOUT,
                                          self.tcp_user_timeout_ms)
                    except OSError:
                        pass
                # 内核保活：比默认 2 小时快得多，能发现半死连接
                client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                for opt, val in (('TCP_KEEPIDLE', 10),
                                 ('TCP_KEEPINTVL', 5),
                                 ('TCP_KEEPCNT', 3)):
                    if hasattr(socket, opt):
                        try:
                            client.setsockopt(socket.IPPROTO_TCP,
                                              getattr(socket, opt), val)
                        except OSError:
                            pass
                client.setblocking(False)
                client_fd = client.fileno()

                self.get_logger().info(f'来自{client_address}的连接已建立')
                self.stat_sessions += 1

                to_master = bytearray()   # 雷达 → ydlidar 驱动
                to_client = bytearray()   # ydlidar 驱动 → 雷达
                disconnect_reason = 'unknown'
                # 只计时「雷达→本端」方向：雷达正常时持续出数据，
                # 这个方向静默就等于转接板挂了。
                last_from_client = time.time()
                session_from_client = 0

                poller.register(master, select.POLLIN)
                poller.register(client_fd, select.POLLIN)

                try:
                    while rclpy.ok():
                        for fd, event in poller.poll(200):
                            if event & (select.POLLHUP | select.POLLERR):
                                raise ConnectionResetError(
                                    f'连接异常 (event={event})')

                            if event & select.POLLIN:
                                try:
                                    data = os.read(fd, self.read_chunk)
                                except BlockingIOError:
                                    data = None
                                except OSError as exc:
                                    raise ConnectionResetError(
                                        f'读取失败: {exc}') from exc

                                if data is not None:
                                    if not data:
                                        raise ConnectionResetError(
                                            '对端关闭连接')
                                    if fd == master:
                                        to_client.extend(data)
                                        self.stat_to_client += len(data)
                                    else:
                                        to_master.extend(data)
                                        self.stat_to_master += len(data)
                                        last_from_client = time.time()
                                        session_from_client += len(data)

                        # 立即冲刷，不依赖下一次 POLLOUT，降低转发时延
                        self._flush(master, client_fd, to_master, to_client)

                        # 按需注册 POLLOUT，避免 poll 空转
                        self._update_mask(poller, master, to_master)
                        self._update_mask(poller, client_fd, to_client)

                        if (self.idle_disconnect_sec > 0 and
                                time.time() - last_from_client >
                                self.idle_disconnect_sec):
                            # 区分「一直没数据」与「跑到一半断掉」，
                            # 这两种对应完全不同的排查方向。
                            if session_from_client == 0:
                                raise ConnectionResetError(
                                    '连接后 %.0fs 未收到任何雷达字节'
                                    '（转接板可能本身就没在转发）'
                                    % self.idle_disconnect_sec)
                            raise ConnectionResetError(
                                '雷达字节流中断 %.0fs（已转 %.1fKB 后停止，'
                                '判定转接板假死，断开逼其重连）'
                                % (self.idle_disconnect_sec,
                                   session_from_client / 1024.0))

                        self._maybe_log_stats()
                except ConnectionResetError as exc:
                    disconnect_reason = str(exc)
                except Exception as exc:  # noqa: BLE001
                    disconnect_reason = f'异常: {exc}'
                finally:
                    for fd in (master, client_fd):
                        try:
                            poller.unregister(fd)
                        except Exception:  # noqa: BLE001
                            pass
                    try:
                        client.close()
                    except Exception:  # noqa: BLE001
                        pass
                    # 断开必须留日志：否则雷达转接板掉线时，只能看到
                    # /scan 消失却不知道桥接已经断了。
                    self.get_logger().warn(
                        f'来自{client_address}的连接已断开'
                        f'（{disconnect_reason}），继续等待重连')
                    if session_from_client == 0:
                        self.get_logger().warn(
                            '本次会话雷达方向 0 字节 —— 不是本端丢弃，'
                            '是转接板/雷达侧没出数据。检查雷达供电、'
                            '转接板与雷达的串口线、以及热点信号强度')
                    self._maybe_log_stats(force=True)
        finally:
            try:
                poller.unregister(master)
            except Exception:  # noqa: BLE001
                pass
            server.close()
            os.close(master)
            os.close(slave)


def main():
    rclpy.init()
    node = TcpSocketServerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
