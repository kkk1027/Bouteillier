#ifndef CAMERA_WIDGET_HPP
#define CAMERA_WIDGET_HPP

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QUrl>
#include <QByteArray>

class CameraWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraWidget(QWidget *parent = nullptr);
    ~CameraWidget();

    void setCameraUrl(const QString &ip, int port = 80);
    void startStream(int fps = 15);
    void stopStream();
    void showFps(bool show);
    double getCurrentFps() const { return current_fps_; }

signals:
    void fpsUpdated(double fps);
    void connectionStatusChanged(bool connected, const QString &message);

private slots:
    void onReadyRead();           // MJPEG 数据到达
    void onStreamFinished();      // 流结束或出错
    void updateFps();

private:
    void startMjpegStream();      // 启动 MJPEG 长连接
    void parseMjpegData();        // 解析 MJPEG 分帧数据z
    void displayImage(const QImage &img);

    QLabel *image_label_;
    QNetworkAccessManager *network_manager_;
    QNetworkReply *stream_reply_;  // MJPEG 长连接
    QTimer *fps_timer_;

    QString camera_base_url_;
    bool is_streaming_;
    bool show_fps_;

    // MJPEG 解析缓冲区
    QByteArray buffer_;
    int target_fps_;
    int frame_count_;
    double current_fps_;
    qint64 last_fps_time_;

    void resizeEvent(QResizeEvent *event) override;
};

#endif // CAMERA_WIDGET_HPP
