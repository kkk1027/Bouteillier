#include "camera_widget.hpp"
#include <QVBoxLayout>
#include <QPainter>
#include <QDateTime>
#include <QDebug>
#include <QNetworkRequest>
#include <QThread>

CameraWidget::CameraWidget(QWidget *parent)
    : QWidget(parent)
    , image_label_(nullptr)
    , network_manager_(new QNetworkAccessManager(this))
    , stream_reply_(nullptr)
    , fps_timer_(nullptr)
    , is_streaming_(false)
    , show_fps_(false)
    , target_fps_(15)
    , frame_count_(0)
    , current_fps_(0.0)
    , last_fps_time_(0)
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    image_label_ = new QLabel(this);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setStyleSheet("QLabel { background-color: black; border: 1px solid #666; }");
    image_label_->setMinimumSize(320, 240);
    image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    image_label_->setText("等待摄像头连接...");
    layout->addWidget(image_label_);

    fps_timer_ = new QTimer(this);
    fps_timer_->setInterval(1000);
    connect(fps_timer_, &QTimer::timeout, this, &CameraWidget::updateFps);
}

CameraWidget::~CameraWidget()
{
    stopStream();
}

void CameraWidget::setCameraUrl(const QString &ip, int port)
{
    camera_base_url_ = QString("http://%1:%2").arg(ip).arg(81);
    qDebug() << "设置摄像头URL:" << camera_base_url_;
}

void CameraWidget::startStream(int fps)
{
    if (is_streaming_) {
        stopStream();
    }

    target_fps_ = qBound(5, fps, 30);

    frame_count_ = 0;
    current_fps_ = 0.0;
    last_fps_time_ = QDateTime::currentMSecsSinceEpoch();
    buffer_.clear();

    image_label_->setText("连接摄像头中...");

    is_streaming_ = true;
    fps_timer_->start();

    startMjpegStream();

    emit connectionStatusChanged(true, QString("视频流已启动 (MJPEG)"));
    qDebug() << "MJPEG 视频流已启动，目标帧率:" << target_fps_;
}

void CameraWidget::stopStream()
{
    if (!is_streaming_) {
        return;
    }

    is_streaming_ = false;
    fps_timer_->stop();

    if (stream_reply_) {
        stream_reply_->abort();
        stream_reply_->deleteLater();
        stream_reply_ = nullptr;
    }

    buffer_.clear();
    image_label_->setText("视频流已停止");

    emit connectionStatusChanged(false, "已停止");
    qDebug() << "视频流已停止";
}

void CameraWidget::showFps(bool show)
{
    show_fps_ = show;
}

void CameraWidget::startMjpegStream()
{
    if (!is_streaming_) {
        return;
    }

    if (stream_reply_) {
        stream_reply_->abort();
        stream_reply_->deleteLater();
        stream_reply_ = nullptr;
    }

    QString url = camera_base_url_ + "/stream";
    QNetworkRequest request(url);
    request.setRawHeader("Accept", "multipart/x-mixed-replace");

    stream_reply_ = network_manager_->get(request);

    connect(stream_reply_, &QNetworkReply::readyRead,
            this, &CameraWidget::onReadyRead);
    connect(stream_reply_, &QNetworkReply::finished,
            this, &CameraWidget::onStreamFinished);
    connect(stream_reply_, &QNetworkReply::errorOccurred,
            [this](QNetworkReply::NetworkError error) {
                qDebug() << "MJPEG 流错误:" << error << stream_reply_->errorString();
                if (is_streaming_) {
                    // 延迟重连
                    QTimer::singleShot(1000, this, &CameraWidget::startMjpegStream);
                }
            });

    qDebug() << "MJPEG 长连接已建立:" << url;
}

void CameraWidget::onReadyRead()
{
    if (!stream_reply_ || !is_streaming_) {
        return;
    }

    buffer_.append(stream_reply_->readAll());
    parseMjpegData();
}

void CameraWidget::parseMjpegData()
{
    // MJPEG 边界标记: --frame 或 --boundary
    // 常见格式: --frame\r\nContent-Type: image/jpeg\r\nContent-Length: xxx\r\n\r\n[数据]\r\n--frame

    QByteArray boundary = "\r\n--";
    int start_pos = 0;

    while (true) {
        // 查找帧头开始位置
        int header_start = buffer_.indexOf(boundary, start_pos);
        if (header_start < 0) break;

        // 查找帧头结束位置（连续两个 \r\n）
        int header_end = buffer_.indexOf("\r\n\r\n", header_start);
        if (header_end < 0) break;

        // 查找 JPEG 数据结束位置（下一个 boundary）
        int data_start = header_end + 4;  // 跳过 \r\n\r\n
        int data_end = buffer_.indexOf(boundary, data_start);
        if (data_end < 0) break;

        // 提取 JPEG 数据
        int jpeg_len = data_end - data_start;
        if (jpeg_len > 0) {
            QByteArray jpegData = buffer_.mid(data_start, jpeg_len);
            QImage img;
            if (img.loadFromData(jpegData, "JPEG")) {
                frame_count_++;
                displayImage(img);
            } else {
                qDebug() << "解码 JPEG 失败，数据大小:" << jpeg_len;
            }
        }

        // 移动到下一个边界位置
        start_pos = data_end;
        // 防止无限循环
        if (start_pos >= buffer_.size()) break;
    }

    // 保留剩余数据（可能是不完整的帧）
    if (start_pos > 0) {
        buffer_ = buffer_.mid(start_pos);
    }
}

void CameraWidget::displayImage(const QImage &img)
{
    if (!is_streaming_) return;

    QPixmap pixmap = QPixmap::fromImage(img);
    QSize labelSize = image_label_->size();
    if (labelSize.width() > 0 && labelSize.height() > 0) {
        pixmap = pixmap.scaled(labelSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }

    // 绘制帧率文字
    if (show_fps_ && current_fps_ > 0) {
        QPainter painter(&pixmap);
        painter.setPen(Qt::white);
        painter.setBrush(QColor(0, 0, 0, 150));
        QRect fpsRect(5, 5, 60, 20);
        painter.drawRect(fpsRect);
        painter.setPen(Qt::green);
        painter.drawText(fpsRect, Qt::AlignCenter, QString("%1 fps").arg(current_fps_, 0, 'f', 1));
    }

    image_label_->setPixmap(pixmap);
}

void CameraWidget::onStreamFinished()
{
    qDebug() << "MJPEG 流连接断开";

    if (is_streaming_ && stream_reply_) {
        // 检查是否有错误
        if (stream_reply_->error() != QNetworkReply::NoError) {
            qDebug() << "流错误:" << stream_reply_->errorString();
        }

        stream_reply_->deleteLater();
        stream_reply_ = nullptr;

        // 自动重连
        if (is_streaming_) {
            qDebug() << "3秒后尝试重连...";
            QTimer::singleShot(3000, this, &CameraWidget::startMjpegStream);
        }
    }
}

void CameraWidget::updateFps()
{
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    qint64 elapsed = now - last_fps_time_;

    if (elapsed > 0) {
        current_fps_ = static_cast<double>(frame_count_) * 1000.0 / elapsed;
        emit fpsUpdated(current_fps_);

        // 动态调整显示：如果帧率超过目标，可以适当降低画质，这里先只打日志
        if (current_fps_ < target_fps_ * 0.7) {
            qDebug() << "当前帧率较低:" << current_fps_ << "fps";
        }

        frame_count_ = 0;
        last_fps_time_ = now;
    }
}

void CameraWidget::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    QPixmap currentPixmap = image_label_->pixmap(Qt::ReturnByValue);
    if (!currentPixmap.isNull()) {
        QSize labelSize = image_label_->size();
        if (labelSize.width() > 0 && labelSize.height() > 0) {
            QPixmap scaled = currentPixmap.scaled(labelSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
            image_label_->setPixmap(scaled);
        }
    }
}
