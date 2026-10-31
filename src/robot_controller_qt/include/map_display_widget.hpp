#ifndef MAP_DISPLAY_WIDGET_HPP
#define MAP_DISPLAY_WIDGET_HPP

#include <QWidget>
#include <QImage>
#include <QPointF>
#include <QColor>
#include <QVector>

class MapDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MapDisplayWidget(QWidget *parent = nullptr);
    ~MapDisplayWidget() = default;
    void clearMap();
void setTestRobotPosition(float x, float y, float theta = 0);
    // 地图相关方法
    void setMapImage(const QImage& map_image);
    void setMapInfo(float resolution, float origin_x, float origin_y);
 void drawCoordinateTest();
    // 机器人位姿方法
    void setRobotPose(float x, float y, float theta);
    void drawReferencePoints(QPainter& painter);

    // 轨迹方法
    void clearTrajectory();
    void setTrajectoryColor(const QColor& color);
    
    // 视图控制
    void resetView();
    void setAutoFollowRobot(bool enable);

public slots:
    void updateMap(const QImage& map_image, float resolution, float origin_x, float origin_y);
    void updateRobotPose(float x, float y, float theta);

signals:
    void viewChanged();
    void robotPoseUpdated(float x, float y, float theta);
    void mapDisplayed();
    void mapRendered();  // 地图真正渲染完成信号
protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    // 坐标转换方法
    QPointF worldToPixel(float world_x, float world_y) const;
    QPointF pixelToWorld(float pixel_x, float pixel_y) const;
    void updateTransform();
    void ensureRobotVisible();
    bool first_render_flag_ = true;
    // 绘制方法
    void drawMap(QPainter& painter);
    void drawTrajectory(QPainter& painter);
    void drawRobot(QPainter& painter);
    void drawGrid(QPainter& painter);
    void drawCoordinateSystem(QPainter& painter);

    // 地图数据
    QImage map_image_;
    bool has_map_ = false;
    
    // 地图信息
    float map_resolution_ = 0.05f;  // 米/像素
    float map_origin_x_ = 0.0f;     // 地图原点X（米）
    float map_origin_y_ = 0.0f;     // 地图原点Y（米）
    int map_width_pixels_ = 0;      // 地图宽度（像素）
    int map_height_pixels_ = 0;     // 地图高度（像素）

    // 机器人数据
    float robot_x_ = 0.0f;          // 世界坐标X（米）
    float robot_y_ = 0.0f;          // 世界坐标Y（米）
    float robot_theta_ = 0.0f;      // 朝向（弧度）
    bool has_robot_pose_ = false;

    // 轨迹数据
    QVector<QPointF> trajectory_points_;  // 世界坐标点
    QColor trajectory_color_ = QColor(255, 0, 0, 150);  // 半透明红色
    int max_trajectory_points_ = 1000;

    // 视图变换参数
    float scale_ = 1.0f;
    float translate_x_ = 0.0f;
    float translate_y_ = 0.0f;
    // 交互状态
    bool is_panning_ = false;
    QPoint last_mouse_pos_;
    
    // 视图控制
    bool auto_follow_robot_ = true;
    
    // 绘制参数
    const float robot_radius_ = 0.2f;  // 米
    const float robot_direction_length_ = 0.3f;  // 米
    const QColor robot_color_ = Qt::black;
    const QColor background_color_ = QColor(240, 240, 240);
    const QColor grid_color_ = QColor(200, 200, 200, 100);
};

#endif // MAP_DISPLAY_WIDGET_HPP
