#include "map_display_widget.hpp"
#include <QPainter>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QDebug>
#include <cmath>
#include <QPainterPath>
#include <QThread>

MapDisplayWidget::MapDisplayWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(400, 400);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);
}

// ==================== 公有方法 ====================
void MapDisplayWidget::setMapImage(const QImage& map_image)
{
    if (map_image.isNull()) {
        qWarning() << "MapDisplayWidget: 接收到空的地图图像";
        return;
    }
    first_render_flag_ = true;
    qDebug() << "setMapImage: 接收到地图图像，尺寸: "
             << map_image.width() << "x" << map_image.height()
             << "格式:" << map_image.format()
             << "深度:" << map_image.depth();

    // 检查图像格式
    if (map_image.format() == QImage::Format_Grayscale8) {
        qDebug() << "图像格式正确: Format_Grayscale8";
    } else {
        qDebug() << "图像格式可能需要转换:" << map_image.format();
        // 如果需要，转换为合适的格式
    }

    map_image_ = map_image;
    map_width_pixels_ = map_image.width();
    map_height_pixels_ = map_image.height();
    has_map_ = true;
    static bool firstRender = true;
    qDebug() << "调用 resetView()...";
    // 重置视图以显示整个地图
    resetView();

    qDebug() << "调用 update() 请求重绘...";
    update();  // 请求重绘

    qDebug() << "MapDisplayWidget: 地图已设置";
}
void MapDisplayWidget::setMapInfo(float resolution, float origin_x, float origin_y)
{
    if (resolution <= 0.0f) {
        qWarning() << "Invalid map resolution:" << resolution;
        return;
    }

    map_resolution_ = resolution;
    map_origin_x_ = origin_x;
    map_origin_y_ = origin_y;
    updateTransform();
}

void MapDisplayWidget::setRobotPose(float x, float y, float theta)
{
    qDebug() << "设置机器人位姿: (" << x << "," << y << ") θ=" << theta;

    robot_x_ = x;
    robot_y_ = y;
    robot_theta_ = theta;
    has_robot_pose_ = true;

    // 添加到轨迹
    trajectory_points_.append(QPointF(x, y));
    qDebug() << "轨迹点数量:" << trajectory_points_.size();

    // 限制轨迹点数量
    if (trajectory_points_.size() > max_trajectory_points_) {
        trajectory_points_.removeFirst();
    }

    // 自动跟随机器人
    if (auto_follow_robot_) {
        qDebug() << "自动跟随机器人...";
        ensureRobotVisible();
    }

    update();
    emit robotPoseUpdated(x, y, theta);
    qDebug() << "机器人位姿更新完成，请求重绘";
}
void MapDisplayWidget::clearTrajectory()
{
    trajectory_points_.clear();
    update();
}

void MapDisplayWidget::setTrajectoryColor(const QColor& color)
{
    trajectory_color_ = color;
    update();
}

void MapDisplayWidget::resetView()
{
    if (!has_map_) {
        scale_ = 1.0f;
        translate_x_ = 0.0f;
        translate_y_ = 0.0f;
        return;
    }

    // 计算适合窗口的地图显示比例
    float width_ratio = static_cast<float>(width()) / map_width_pixels_;
    float height_ratio = static_cast<float>(height()) / map_height_pixels_;
    scale_ = std::min(width_ratio, height_ratio) * 0.9f;  // 留10%边距

    // 居中显示
    translate_x_ = (width() - map_width_pixels_ * scale_) / 2.0f;
    translate_y_ = (height() - map_height_pixels_ * scale_) / 2.0f;

    update();
}

void MapDisplayWidget::setAutoFollowRobot(bool enable)
{
    auto_follow_robot_ = enable;
    if (enable && has_robot_pose_) {
        ensureRobotVisible();
    }
}

// ==================== 公有槽 ====================

void MapDisplayWidget::updateMap(const QImage& map_image, float resolution,
                                 float origin_x, float origin_y)
{
    qDebug() << "=== updateMap 槽函数被调用 ===";
    qDebug() << "图像大小:" << map_image.size();
    qDebug() << "图像格式:" << map_image.format();
    qDebug() << "分辨率:" << resolution;
    qDebug() << "原点(" << origin_x << "," << origin_y << ")";

    // 检查图像是否有效
    if (map_image.isNull()) {
        qWarning() << "updateMap: 接收到空图像!";
        return;
    }

    // 保存图像用于调试
    static int count = 0;
    QString filename = QString("/tmp/qt_map_debug_%1.png").arg(count++);
    if (map_image.save(filename)) {
        qDebug() << "地图图像已保存到:" << filename;
    }

    setMapInfo(resolution, origin_x, origin_y);
    setMapImage(map_image);

    qDebug() << "=== updateMap 完成 ===";
}
void MapDisplayWidget::updateRobotPose(float x, float y, float theta)
{
    qDebug() << "=== updateRobotPose 被调用 ===";
    qDebug() << "传入参数: x=" << x << " y=" << y << " theta=" << theta;
    qDebug() << "当前线程: " << QThread::currentThread();
    qDebug() << "之前的值: x=" << robot_x_ << " y=" << robot_y_ << " theta=" << robot_theta_;

    // 检查是否真的有变化
    bool changed = (fabs(robot_x_ - x) > 0.001 ||
                    fabs(robot_y_ - y) > 0.001 ||
                    fabs(robot_theta_ - theta) > 0.001);

    qDebug() << "值是否变化: " << changed;

    // 更新值
    robot_x_ = x;
    robot_y_ = y;
    robot_theta_ = theta;
    has_robot_pose_ = true;

    qDebug() << "更新后的值: x=" << robot_x_ << " y=" << robot_y_ << " theta=" << robot_theta_;
    qDebug() << "has_robot_pose_ 设置为: " << has_robot_pose_;

    // 添加到轨迹
    trajectory_points_.append(QPointF(x, y));
    qDebug() << "轨迹点数量:" << trajectory_points_.size();

    update();

    emit robotPoseUpdated(x, y, theta);
    qDebug() << "=== updateRobotPose 完成 ===";
}
// ==================== 事件处理 ====================
void MapDisplayWidget::paintEvent(QPaintEvent *event)
{
    qDebug() << "=== paintEvent 被调用 ===";
    qDebug() << "Widget大小:" << size();
    qDebug() << "has_map_:" << has_map_;
    qDebug() << "has_robot_pose_:" << has_robot_pose_;
    qDebug() << "机器人坐标值: x=" << robot_x_ << " y=" << robot_y_ << " theta=" << robot_theta_;
    static bool firstRender = true;
       if (has_map_ && firstRender) {
           firstRender = false;
           emit mapRendered();
       }    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // 绘制背景
    painter.fillRect(rect(), background_color_);

    if (!has_map_) {
        // 没有地图时显示提示
        painter.setPen(Qt::gray);
        painter.drawText(rect(), Qt::AlignCenter, "等待地图数据...");
        qDebug() << "paintEvent: 没有地图数据，显示等待消息";
        return;
    }

    qDebug() << "paintEvent: 开始绘制地图，地图大小:"
             << map_image_.size() << "缩放:" << scale_;

    // 保存当前变换
    painter.save();

    // 应用视图变换
    painter.translate(translate_x_, translate_y_);
    painter.scale(scale_, scale_);

    qDebug() << "绘制地图...";
    // 绘制地图
    drawMap(painter);

    // 总是绘制机器人，不管标志位
    qDebug() << "强制绘制机器人...";
    drawRobot(painter);

    // 绘制轨迹
//    drawTrajectory(painter);
    if (has_map_ && first_render_flag_) {
            first_render_flag_ = false;
            emit mapRendered();
        }
    // 恢复变换
    painter.restore();

    // 绘制坐标系指示
    drawCoordinateSystem(painter);

    qDebug() << "=== paintEvent 完成 ===";
}
//void MapDisplayWidget::drawReferencePoints(QPainter& painter)
//{
//    painter.save();

//    // 地图的4个角（像素坐标）
//    // 注意：这是在变换后的坐标系中，所以使用像素坐标
////rviz原点坐标
//    painter.setBrush(Qt::black);
//    QPointF world_origin_pixel = worldToPixel(0, 0);
//       painter.drawEllipse(world_origin_pixel, 8, 8);  // 比参考点大一点
//       painter.drawText(world_origin_pixel + QPointF(10, -10),
//                       QString("世界原点(0,0)\n像素(%1,%2)")
//                       .arg(world_origin_pixel.x(), 0, 'f', 1)
//                       .arg(world_origin_pixel.y(), 0, 'f', 1));
//    // 左下角 (0, map_height_pixels_)
//    painter.setBrush(Qt::red);
//    painter.drawEllipse(QPointF(173.4,131.8), 5, 5);
//    painter.drawText(QPointF(5, map_height_pixels_ - 5), "左下角(0, h)");

//    // 调试：计算左下角的世界坐标
//    QPointF world_bl = pixelToWorld(0, map_height_pixels_);
//    qDebug() << "左下角(红): 像素(0," << map_height_pixels_ << ") -> 世界("
//             << world_bl.x() << "," << world_bl.y() << ")米";

//    // 左上角 (0, 0)
//    painter.setBrush(Qt::green);
//    painter.drawEllipse(QPointF(0, 0), 5, 5);
//    painter.drawText(QPointF(5, 15), "左上角(0, 0)");

//    // 调试：计算左上角的世界坐标
//    QPointF world_tl = pixelToWorld(0, 0);
//    qDebug() << "左上角(绿): 像素(0,0) -> 世界("
//             << world_tl.x() << "," << world_tl.y() << ")米";

//    // 右上角 (map_width_pixels_, 0)
//    painter.setBrush(Qt::blue);
//    painter.drawEllipse(QPointF(map_width_pixels_, 0), 5, 5);
//    painter.drawText(QPointF(map_width_pixels_ - 60, 15), "右上角(w, 0)");

//    // 调试：计算右上角的世界坐标
//    QPointF world_tr = pixelToWorld(map_width_pixels_, 0);
//    qDebug() << "右上角(蓝): 像素(" << map_width_pixels_ << ",0) -> 世界("
//             << world_tr.x() << "," << world_tr.y() << ")米";

//    // 右下角 (map_width_pixels_, map_height_pixels_)
//    painter.setBrush(Qt::yellow);
//    painter.drawEllipse(QPointF(map_width_pixels_, map_height_pixels_), 5, 5);
//    painter.drawText(QPointF(map_width_pixels_ - 60, map_height_pixels_ - 5), "右下角(w, h)");

//    // 调试：计算右下角的世界坐标
//    QPointF world_br = pixelToWorld(map_width_pixels_, map_height_pixels_);
//    qDebug() << "右下角(黄): 像素(" << map_width_pixels_ << ","
//             << map_height_pixels_ << ") -> 世界("
//             << world_br.x() << "," << world_br.y() << ")米";

//    // 地图中心
//    painter.setBrush(Qt::magenta);
//    QPointF center(map_width_pixels_ / 2.0, map_height_pixels_ / 2.0);
//    painter.drawEllipse(center, 5, 5);
//    painter.drawText(center + QPointF(5, -5), "地图中心");

//    // 调试：计算地图中心的世界坐标
//    QPointF world_center = pixelToWorld(map_width_pixels_/2.0, map_height_pixels_/2.0);
//    qDebug() << "地图中心(洋红): 像素(" << map_width_pixels_/2.0 << ","
//             << map_height_pixels_/2.0 << ") -> 世界("
//             << world_center.x() << "," << world_center.y() << ")米";

//    painter.restore();

//    // 同时输出地图基本信息（只输出一次）
//    static bool firstCall = true;
//    if (firstCall && has_map_) {
//        firstCall = false;
//        qDebug() << "\n=== 地图基本信息 ===";
//        qDebug() << "地图尺寸: " << map_width_pixels_ << "x" << map_height_pixels_ << "像素";
//        qDebug() << "地图分辨率: " << map_resolution_ << "米/像素";
//        qDebug() << "地图原点: (" << map_origin_x_ << ", " << map_origin_y_ << ")米";
//        qDebug() << "视图缩放: " << scale_;
//        qDebug() << "视图平移: (" << translate_x_ << ", " << translate_y_ << ")";
//        qDebug() << "=== 地图参考点世界坐标 ===";
//    }
//}
void MapDisplayWidget::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    if (auto_follow_robot_) {
        ensureRobotVisible();
    }
}

void MapDisplayWidget::wheelEvent(QWheelEvent *event)
{
    float zoom_factor = 1.1f;

    // 获取鼠标位置（相对于widget）
    QPointF mouse_pos = event->position();

    // 计算鼠标在变换前的世界坐标
    QPointF mouse_before = pixelToWorld(mouse_pos.x(), mouse_pos.y());

    // 调整缩放比例
    if (event->angleDelta().y() > 0) {
        // 放大
        scale_ *= zoom_factor;
    } else {
        // 缩小
        scale_ /= zoom_factor;
    }

    // 限制缩放范围
    scale_ = qBound(0.1f, scale_, 20.0f);

    // 计算鼠标在变换后的位置
    QPointF mouse_after = worldToPixel(mouse_before.x(), mouse_before.y());

    // 调整平移，使鼠标点保持在同一位置
    translate_x_ += mouse_pos.x() - mouse_after.x();
    translate_y_ += mouse_pos.y() - mouse_after.y();

    update();
    emit viewChanged();
}

void MapDisplayWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        is_panning_ = true;
        last_mouse_pos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void MapDisplayWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (is_panning_) {
        QPoint delta = event->pos() - last_mouse_pos_;
        translate_x_ += delta.x();
        translate_y_ += delta.y();
        last_mouse_pos_ = event->pos();
        update();
        emit viewChanged();
    }
}

void MapDisplayWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        is_panning_ = false;
        setCursor(Qt::ArrowCursor);
    }
}

// ==================== 私有方法 ====================
QPointF MapDisplayWidget::worldToPixel(float world_x, float world_y) const
{
    // 使用地图原点进行正确的坐标转换
    float pixel_x = (world_x - map_origin_x_) / map_resolution_;
    float pixel_y = (world_y - map_origin_y_) / map_resolution_;

    return QPointF(pixel_x, pixel_y);
}

QPointF MapDisplayWidget::pixelToWorld(float pixel_x, float pixel_y) const
{
    // 去除视图变换，得到原始地图像素坐标
    float map_pixel_x = (pixel_x - translate_x_) / scale_;
    float map_pixel_y = (pixel_y - translate_y_) / scale_;

    // 像素坐标转世界坐标：世界坐标 = 像素坐标 * 分辨率 + 地图原点
    float world_x = map_pixel_x * map_resolution_ + map_origin_x_;
    float world_y = map_pixel_y * map_resolution_ + map_origin_y_;

    return QPointF(world_x, world_y);
}

void MapDisplayWidget::drawCoordinateTest()
{
    qDebug() << "=== 坐标系测试开始 ===";
    qDebug() << "地图尺寸:" << map_width_pixels_ << "x" << map_height_pixels_;
    qDebug() << "地图原点:(" << map_origin_x_ << "," << map_origin_y_ << ")";

    // 测试四个角点
    QPointF corners[4] = {
        QPointF(map_origin_x_, map_origin_y_),  // 左下角
        QPointF(map_origin_x_ + map_width_pixels_ * map_resolution_, map_origin_y_),  // 右下角
        QPointF(map_origin_x_, map_origin_y_ + map_height_pixels_ * map_resolution_),  // 左上角
        QPointF(map_origin_x_ + map_width_pixels_ * map_resolution_,
                map_origin_y_ + map_height_pixels_ * map_resolution_)  // 右上角
    };

    QString corner_names[4] = {"左下角", "右下角", "左上角", "右上角"};

    for (int i = 0; i < 4; i++) {
        QPointF pixel = worldToPixel(corners[i].x(), corners[i].y());
        qDebug() << corner_names[i] << "世界(" << corners[i].x() << "," << corners[i].y()
                 << ") -> 屏幕(" << pixel.x() << "," << pixel.y() << ")";
    }
}
void MapDisplayWidget::updateTransform()
{
    // 如果需要，可以在这里更新变换矩阵
}
void MapDisplayWidget::setTestRobotPosition(float x, float y, float theta)
{
    qDebug() << "=== 测试机器人位置 ===";
    qDebug() << "设置测试位置: (" << x << ", " << y << ") θ=" << theta;

    // 直接设置位姿，绕过ROS
    robot_x_ = x;
    robot_y_ = y;
    robot_theta_ = theta;
    has_robot_pose_ = true;

    // 立即测试坐标转换
    QPointF pixel_pos = worldToPixel(x, y);
    qDebug() << "转换结果 - 屏幕坐标: (" << pixel_pos.x() << ", " << pixel_pos.y() << ")";

    update();  // 强制重绘
}
void MapDisplayWidget::ensureRobotVisible()
{
    if (!has_robot_pose_) return;

    // 将机器人位置转换到屏幕坐标
    QPointF robot_screen = worldToPixel(robot_x_, robot_y_);

    // 计算可见区域（留20%边距）
    float margin = 0.2f;
    int visible_left = width() * margin;
    int visible_right = width() * (1.0f - margin);
    int visible_top = height() * margin;
    int visible_bottom = height() * (1.0f - margin);

    // 检查机器人是否在可见区域内
    if (robot_screen.x() < visible_left || robot_screen.x() > visible_right ||
        robot_screen.y() < visible_top || robot_screen.y() > visible_bottom) {

        // 将机器人移动到中心
        QPointF center_screen(width() / 2.0f, height() / 2.0f);
        QPointF robot_map_pixel = worldToPixel(robot_x_, robot_y_);

        // 调整平移，使机器人在中心
        translate_x_ = center_screen.x() - robot_map_pixel.x() * scale_;
        translate_y_ = center_screen.y() - robot_map_pixel.y() * scale_;

        update();
    }
}

// ==================== 绘制方法 ====================

void MapDisplayWidget::drawMap(QPainter& painter)
{
    if (map_image_.isNull()) {
        qWarning() << "drawMap: 地图图像为空!";
        return;
    }
    // 保存当前状态
       painter.save();

       // 方法A：左右翻转（绕Y轴翻转）
       painter.translate(map_width_pixels_, 0);  // 平移到右侧
       painter.scale(-1, 1);  // X方向翻转（负数缩放）
    qDebug() << "drawMap: 绘制图像，位置:(0,0) 大小:"
             << map_image_.width() << "x" << map_image_.height();

    // 直接绘制图像
    painter.drawImage(QRect(0, 0, map_image_.width(), map_image_.height()),
                     map_image_);

    qDebug() << "drawMap: 图像绘制完成";
}
void MapDisplayWidget::drawTrajectory(QPainter& painter)
{
    if (trajectory_points_.size() < 2) return;

    painter.save();
    QPen pen(trajectory_color_);
    pen.setWidth(2);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    painter.setPen(pen);

    // 绘制轨迹线
    QPainterPath path;
    QPointF first_point = worldToPixel(trajectory_points_.first().x(),
                                       trajectory_points_.first().y());
    path.moveTo(first_point);

    for (int i = 1; i < trajectory_points_.size(); ++i) {
        QPointF point = worldToPixel(trajectory_points_[i].x(),
                                     trajectory_points_[i].y());
        path.lineTo(point);
    }

    painter.drawPath(path);
    painter.restore();
}

void MapDisplayWidget::drawRobot(QPainter& painter)
{
    painter.save();

    // 即使 has_robot_pose_ 为 false，也使用当前存储的值
    qDebug() << "=== drawRobot 开始 ===";
    qDebug() << "has_robot_pose_:" << has_robot_pose_;
    qDebug() << "机器人坐标: (" << robot_x_ << "," << robot_y_ << ")";

    // 强制绘制（即使标志为 false）
    if (robot_x_ == 0.0f && robot_y_ == 0.0f && !has_robot_pose_) {
        qDebug() << "警告：机器人坐标为原点，可能是默认值";
    }

    // 绘制机器人（黑色圆点）
    QPointF robot_pixel = worldToPixel(robot_x_, robot_y_);
    qDebug() << "机器人像素坐标: (" << robot_pixel.x() << "," << robot_pixel.y() << ")";

    // 计算圆点半径（在世界坐标系中固定大小）
    float robot_radius_pixels = robot_radius_ / map_resolution_;
    qDebug() << "机器人半径像素: " << robot_radius_pixels;

    painter.setBrush(robot_color_);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(robot_pixel, robot_radius_pixels, robot_radius_pixels);

    // 绘制方向指示
    float end_x = robot_x_ + robot_direction_length_ * cos(robot_theta_);
    float end_y = robot_y_ + robot_direction_length_ * sin(robot_theta_);
    QPointF end_pixel = worldToPixel(end_x, end_y);

    QPen direction_pen(Qt::white);
    direction_pen.setWidth(2);
    painter.setPen(direction_pen);
    painter.drawLine(robot_pixel, end_pixel);

    painter.restore();
    qDebug() << "=== drawRobot 完成 ===";
}
void MapDisplayWidget::drawGrid(QPainter& painter)
{
    // 如果需要网格，可以在这里实现
    Q_UNUSED(painter);
}

void MapDisplayWidget::drawCoordinateSystem(QPainter& painter)
{
    painter.save();

    // 在右下角绘制坐标系指示
    int indicator_size = 80;
    int margin = 10;
    QRect indicator_rect(width() - indicator_size - margin,
                        height() - indicator_size - margin,
                        indicator_size, indicator_size);

    // 绘制背景
    painter.setBrush(QColor(255, 255, 255, 200));
    painter.setPen(Qt::NoPen);
    painter.drawRect(indicator_rect);

    // 绘制坐标轴
    int center_x = indicator_rect.center().x();
    int center_y = indicator_rect.center().y();
    int axis_length = indicator_size / 2 - 10;

    // X轴（红色）
    painter.setPen(QPen(Qt::red, 2));
    painter.drawLine(center_x, center_y, center_x + axis_length, center_y);
    painter.drawText(center_x + axis_length, center_y, "X");

    // Y轴（绿色）
    painter.setPen(QPen(Qt::green, 2));
    painter.drawLine(center_x, center_y, center_x, center_y - axis_length);
    painter.drawText(center_x, center_y - axis_length, "Y");

    // 添加比例尺文字
    QString scale_text = QString("比例: 1:%1").arg(1.0 / (map_resolution_ * scale_), 0, 'f', 2);
    painter.setPen(Qt::black);
    painter.drawText(indicator_rect.x() + 5, indicator_rect.y() + 15, scale_text);

    if (has_robot_pose_) {
        QString pose_text = QString("位置: (%1, %2)")
                           .arg(robot_x_, 0, 'f', 2)
                           .arg(robot_y_, 0, 'f', 2);
        painter.drawText(indicator_rect.x() + 5, indicator_rect.y() + 30, pose_text);
    }

    painter.restore();
}
void MapDisplayWidget::clearMap()
{
    qDebug() << "MapDisplayWidget::clearMap() - 清空地图显示";

    // 清空地图图像
    map_image_ = QImage();
    has_map_ = false;
    first_render_flag_ = true;
    // 清空机器人位姿
    has_robot_pose_ = false;
    robot_x_ = 0.0f;
    robot_y_ = 0.0f;
    robot_theta_ = 0.0f;

    // 清空轨迹
    trajectory_points_.clear();

    // 重置首次渲染标志（以便下次重新加载时能正确触发 mapRendered 信号）
    // 如果之前使用的是 static 变量，需要改为成员变量

    // 强制重绘，显示空白界面
    update();

    qDebug() << "地图显示已清空，请求重绘";
}
