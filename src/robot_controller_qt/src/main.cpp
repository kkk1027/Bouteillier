// main.cpp - 添加航点队列导航和一键返航功能
#include "camera_widget.hpp"
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QMessageBox>
#include <QTimer>
#include <QFrame>
#include <QGroupBox>
#include <QFont>
#include <QTabWidget>
#include <QSplitter>
#include <QCheckBox>
#include <QComboBox>
#include <QSpinBox>
#include <QDebug>
#include <QStyle>
#include <QPalette>
#include <QLinearGradient>
#include <QFontDatabase>
#include <QPropertyAnimation>
#include <QEasingCurve>
#include <QGraphicsDropShadowEffect>
#include <QDateTime>
#include <QListWidget>
#include <QListWidgetItem>

#include "launch_manager.hpp"
#include "manual_controller.hpp"
#include "ros_map_bridge.hpp"
#include "map_display_widget.hpp"

// 自定义样式辅助函数
void applyShadow(QWidget *widget, int blurRadius = 8, int offset = 0, const QColor &color = QColor(0, 0, 0, 25))
{
    auto *shadow = new QGraphicsDropShadowEffect(widget);
    shadow->setBlurRadius(blurRadius);
    shadow->setOffset(offset);
    shadow->setColor(color);
    widget->setGraphicsEffect(shadow);
}

// 创建圆角按钮
QPushButton* createStyledButton(const QString &text, const QString &bgColor = "#2196F3",
                                 const QString &hoverColor = "#1976D2", bool isLarge = false)
{
    QPushButton *btn = new QPushButton(text);
    btn->setFixedHeight(isLarge ? 45 : 36);
    btn->setStyleSheet(QString(
        "QPushButton {"
        "    background-color: %1;"
        "    color: white;"
        "    border: none;"
        "    border-radius: 6px;"
        "    font-weight: bold;"
        "    padding: 6px 12px;"
        "    font-size: %2px;"
        "}"
        "QPushButton:hover {"
        "    background-color: %3;"
        "}"
        "QPushButton:pressed {"
        "    background-color: %4;"
        "}"
        "QPushButton:disabled {"
        "    background-color: #cccccc;"
        "    color: #888888;"
        "}"
    ).arg(bgColor).arg(isLarge ? "13" : "11").arg(hoverColor).arg(bgColor));
    return btn;
}

// 创建状态标签
QLabel* createStatusLabel(const QString &text, const QString &color = "#757575")
{
    QLabel *label = new QLabel(text);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet(QString(
        "QLabel {"
        "    background-color: #f5f5f5;"
        "    border-radius: 5px;"
        "    padding: 6px;"
        "    color: %1;"
        "    font-weight: 500;"
        "    border: 1px solid #e0e0e0;"
        "    font-size: 12px;"
        "}"
    ).arg(color));
    return label;
}

// 创建分组框
QGroupBox* createStyledGroupBox(const QString &title)
{
    QGroupBox *groupBox = new QGroupBox(title);
    groupBox->setStyleSheet(
        "QGroupBox {"
        "    font-weight: bold;"
        "    border: 2px solid #e0e0e0;"
        "    border-radius: 8px;"
        "    margin-top: 12px;"
        "    padding-top: 12px;"
        "    background-color: #ffffff;"
        "    font-size: 13px;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    left: 12px;"
        "    padding: 0 8px 0 8px;"
        "    color: #2196F3;"
        "}"
    );
    return groupBox;
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // 设置应用程序样式
    app.setStyle("Fusion");

    // 设置调色板
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(245, 245, 245));
    palette.setColor(QPalette::WindowText, QColor(33, 33, 33));
    palette.setColor(QPalette::Base, QColor(255, 255, 255));
    palette.setColor(QPalette::AlternateBase, QColor(245, 245, 245));
    palette.setColor(QPalette::ToolTipBase, QColor(255, 255, 255));
    palette.setColor(QPalette::ToolTipText, QColor(33, 33, 33));
    palette.setColor(QPalette::Text, QColor(33, 33, 33));
    palette.setColor(QPalette::Button, QColor(255, 255, 255));
    palette.setColor(QPalette::ButtonText, QColor(33, 33, 33));
    palette.setColor(QPalette::BrightText, Qt::red);
    palette.setColor(QPalette::Link, QColor(33, 150, 243));
    palette.setColor(QPalette::Highlight, QColor(33, 150, 243));
    palette.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
    app.setPalette(palette);

    // 主窗口 - 缩小整体尺寸
    QWidget window;
    window.setWindowTitle("Bouteillier Home-家居巡检机器人系统");
    window.resize(1200, 800);
    window.setStyleSheet("background-color: #f5f5f5;");

    // 主布局 - 减小边距
    QVBoxLayout *mainLayout = new QVBoxLayout(&window);
    mainLayout->setSpacing(12);
    mainLayout->setContentsMargins(15, 15, 15, 15);

    // === 标题栏（缩小） ===
    QWidget *titleWidget = new QWidget();
    titleWidget->setStyleSheet("background-color: white; border-radius: 12px;");
    applyShadow(titleWidget, 12, 0, QColor(0, 0, 0, 15));

    QHBoxLayout *titleLayout = new QHBoxLayout(titleWidget);
    titleLayout->setContentsMargins(15, 12, 15, 12);

    QLabel *iconLabel = new QLabel("🤖");
    QFont iconFont = iconLabel->font();
    iconFont.setPointSize(20);
    iconLabel->setFont(iconFont);

    QLabel *titleLabel = new QLabel("Bouteillier Home-家居巡检机器人系统");
    titleLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    QFont titleFont = titleLabel->font();
    titleFont.setPointSize(18);
    titleFont.setBold(true);
    titleFont.setFamily("Microsoft YaHei");
    titleLabel->setFont(titleFont);
    titleLabel->setStyleSheet("color: #2196F3;");

    QLabel *versionLabel = new QLabel("v1.0.0");
    versionLabel->setStyleSheet("color: #9e9e9e; padding: 3px 10px; background-color: #f0f0f0; border-radius: 10px; font-size: 11px;");

    titleLayout->addWidget(iconLabel);
    titleLayout->addWidget(titleLabel);
    titleLayout->addStretch();
    titleLayout->addWidget(versionLabel);

    mainLayout->addWidget(titleWidget);

    QLabel *batteryIcon = new QLabel("🔋");
    batteryIcon->setStyleSheet("font-size: 16px;");

    QLabel *batteryLabel = new QLabel("--%");
    batteryLabel->setStyleSheet(
        "QLabel {"
        "    color: #333;"
        "    font-weight: bold;"
        "    font-size: 12px;"
        "    background-color: #f0f0f0;"
        "    border-radius: 12px;"
        "    padding: 4px 12px;"
        "}"
    );

    titleLayout->addWidget(batteryIcon);
    titleLayout->addWidget(batteryLabel);
    // === 主内容区域 ===
    QSplitter *mainSplitter = new QSplitter(Qt::Horizontal);
    mainSplitter->setChildrenCollapsible(false);
    mainSplitter->setStyleSheet("QSplitter::handle { background-color: #e0e0e0; width: 2px; }");

    // === 左侧控制面板 ===
    QWidget *leftPanel = new QWidget();
    leftPanel->setStyleSheet("background-color: transparent;");
    QVBoxLayout *leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setSpacing(12);
    leftLayout->setContentsMargins(0, 0, 8, 0);

    // === 连接状态卡片（缩小） ===
    QWidget *connectionCard = new QWidget();
    connectionCard->setStyleSheet("background-color: white; border-radius: 12px;");
    applyShadow(connectionCard, 8, 0, QColor(0, 0, 0, 12));

    QVBoxLayout *connectionCardLayout = new QVBoxLayout(connectionCard);
    connectionCardLayout->setSpacing(12);
    connectionCardLayout->setContentsMargins(15, 15, 15, 15);

    QHBoxLayout *connectionTitleLayout = new QHBoxLayout();
    QLabel *connectionIcon = new QLabel("🔌");
    QLabel *connectionTitle = new QLabel("连接状态");
    connectionTitle->setStyleSheet("font-size: 14px; font-weight: bold; color: #333;");
    connectionTitleLayout->addWidget(connectionIcon);
    connectionTitleLayout->addWidget(connectionTitle);
    connectionTitleLayout->addStretch();

    QHBoxLayout *connectionButtonLayout = new QHBoxLayout();
    QPushButton *connectButton = createStyledButton("连接机器人", "#4CAF50", "#388E3C", true);
    QPushButton *cleanupButton = createStyledButton("强制清理", "#f44336", "#d32f2f");

    connectionButtonLayout->addWidget(connectButton, 2);
    connectionButtonLayout->addWidget(cleanupButton, 1);

    QLabel *statusLabel = new QLabel("⚪ 未连接");
    statusLabel->setAlignment(Qt::AlignCenter);
    statusLabel->setStyleSheet(
        "QLabel {"
        "    background-color: #f5f5f5;"
        "    border-radius: 6px;"
        "    padding: 10px;"
        "    font-weight: bold;"
        "    color: #757575;"
        "    font-size: 12px;"
        "}"
    );

    connectionCardLayout->addLayout(connectionTitleLayout);
    connectionCardLayout->addLayout(connectionButtonLayout);
    connectionCardLayout->addWidget(statusLabel);

    leftLayout->addWidget(connectionCard);

    // === 地图信息卡片（缩小） ===
    QWidget *mapInfoCard = new QWidget();
    mapInfoCard->setStyleSheet("background-color: white; border-radius: 12px;");
    applyShadow(mapInfoCard, 8, 0, QColor(0, 0, 0, 12));

    QVBoxLayout *mapInfoCardLayout = new QVBoxLayout(mapInfoCard);
    mapInfoCardLayout->setSpacing(8);
    mapInfoCardLayout->setContentsMargins(15, 15, 15, 15);

    QHBoxLayout *mapInfoTitleLayout = new QHBoxLayout();
    QLabel *mapInfoIcon = new QLabel("🗺️");
    QLabel *mapInfoTitle = new QLabel("地图信息");
    mapInfoTitle->setStyleSheet("font-size: 14px; font-weight: bold; color: #333;");
    mapInfoTitleLayout->addWidget(mapInfoIcon);
    mapInfoTitleLayout->addWidget(mapInfoTitle);
    mapInfoTitleLayout->addStretch();

    QLabel *mapStatusLabel = new QLabel("📡 等待地图数据...");
    mapStatusLabel->setStyleSheet("padding: 6px; background-color: #f5f5f5; border-radius: 5px; font-size: 11px;");

    QLabel *robotPoseLabel = new QLabel("📍 位置: 未知");
    robotPoseLabel->setStyleSheet("padding: 6px; background-color: #f5f5f5; border-radius: 5px; font-size: 11px;");

    mapInfoCardLayout->addLayout(mapInfoTitleLayout);
    mapInfoCardLayout->addWidget(mapStatusLabel);
    mapInfoCardLayout->addWidget(robotPoseLabel);

    leftLayout->addWidget(mapInfoCard);

    // === 功能选项卡（缩小） ===
    QTabWidget *tabWidget = new QTabWidget();
    tabWidget->setStyleSheet(
        "QTabWidget::pane {"
        "    border: none;"
        "    background: transparent;"
        "    margin-top: -1px;"
        "}"
        "QTabBar::tab {"
        "    background-color: #f0f0f0;"
        "    border: none;"
        "    border-top-left-radius: 8px;"
        "    border-top-right-radius: 8px;"
        "    padding: 8px 20px;"
        "    margin-right: 2px;"
        "    font-weight: bold;"
        "    color: #757575;"
        "    font-size: 12px;"
        "}"
        "QTabBar::tab:selected {"
        "    background-color: white;"
        "    color: #2196F3;"
        "    border-bottom: 3px solid #2196F3;"
        "}"
        "QTabBar::tab:hover:!selected {"
        "    background-color: #e0e0e0;"
        "    color: #333;"
        "}"
    );

    // === 选项卡1: 手动控制（完全保持不变，但尺寸缩小） ===
    QWidget *manualTab = new QWidget();
    manualTab->setStyleSheet("background-color: white; border-radius: 0 0 12px 12px;");
    QVBoxLayout *manualTabLayout = new QVBoxLayout(manualTab);
    manualTabLayout->setSpacing(15);
    manualTabLayout->setContentsMargins(15, 15, 15, 15);

    // 速度控制
    QGroupBox *speedGroup = createStyledGroupBox("速度设置");
    QVBoxLayout *speedLayout = new QVBoxLayout(speedGroup);

    QLabel *speedDisplay = new QLabel("速度: 线速 0.20 m/s | 角速 0.50 rad/s");
    speedDisplay->setAlignment(Qt::AlignCenter);
    speedDisplay->setStyleSheet(
        "QLabel {"
        "    background-color: #e3f2fd;"
        "    border-radius: 6px;"
        "    padding: 10px;"
        "    font-weight: bold;"
        "    color: #1976d2;"
        "    font-size: 12px;"
        "}"
    );

    QHBoxLayout *speedButtonLayout = new QHBoxLayout();
    QPushButton *increaseSpeedButton = createStyledButton("➕ 加速", "#FF9800", "#F57C00");
    QPushButton *decreaseSpeedButton = createStyledButton("➖ 减速", "#FF9800", "#F57C00");
    QPushButton *resetSpeedButton = createStyledButton("🔄 重置", "#9C27B0", "#7B1FA2");

    speedButtonLayout->addWidget(increaseSpeedButton);
    speedButtonLayout->addWidget(decreaseSpeedButton);
    speedButtonLayout->addWidget(resetSpeedButton);

    speedLayout->addWidget(speedDisplay);
    speedLayout->addLayout(speedButtonLayout);

    // 方向控制
    QGroupBox *directionGroup = createStyledGroupBox("方向控制");
    QVBoxLayout *directionLayout = new QVBoxLayout(directionGroup);

    QGridLayout *gridLayout = new QGridLayout();
    gridLayout->setSpacing(8);

    QStringList buttonStyles = {
        "QPushButton { background-color: #2196F3; color: white; border: none; border-radius: 30px; font-size: 20px; font-weight: bold; }"
        "QPushButton:hover { background-color: #1976D2; }"
        "QPushButton:pressed { background-color: #0D47A1; }"
        "QPushButton:disabled { background-color: #BDBDBD; }"
    };

    QPushButton *btnUpLeft = new QPushButton("↖");
    QPushButton *btnUp = new QPushButton("↑");
    QPushButton *btnUpRight = new QPushButton("↗");
    QPushButton *btnLeft = new QPushButton("←");
    QPushButton *btnStop = new QPushButton("●");
    QPushButton *btnRight = new QPushButton("→");
    QPushButton *btnDownLeft = new QPushButton("↙");
    QPushButton *btnDown = new QPushButton("↓");
    QPushButton *btnDownRight = new QPushButton("↘");

    QList<QPushButton*> directionButtons = {
        btnUpLeft, btnUp, btnUpRight,
        btnLeft, btnStop, btnRight,
        btnDownLeft, btnDown, btnDownRight
    };

    for (auto btn : directionButtons) {
        btn->setFixedSize(60, 60);
        btn->setEnabled(false);
        btn->setStyleSheet(buttonStyles[0]);
        if (btn == btnStop) {
            btn->setStyleSheet(
                "QPushButton { background-color: #f44336; color: white; border: none; border-radius: 30px; font-size: 24px; }"
                "QPushButton:hover { background-color: #d32f2f; }"
                "QPushButton:pressed { background-color: #b71c1c; }"
                "QPushButton:disabled { background-color: #BDBDBD; }"
            );
        }
    }

    gridLayout->addWidget(btnUpLeft, 0, 0);
    gridLayout->addWidget(btnUp, 0, 1);
    gridLayout->addWidget(btnUpRight, 0, 2);
    gridLayout->addWidget(btnLeft, 1, 0);
    gridLayout->addWidget(btnStop, 1, 1);
    gridLayout->addWidget(btnRight, 1, 2);
    gridLayout->addWidget(btnDownLeft, 2, 0);
    gridLayout->addWidget(btnDown, 2, 1);
    gridLayout->addWidget(btnDownRight, 2, 2);

    directionLayout->addLayout(gridLayout);

    manualTabLayout->addWidget(speedGroup);
    manualTabLayout->addWidget(directionGroup);
    manualTabLayout->addStretch();

    // === 选项卡2: 自主导航（添加航点选择和一键返航功能） ===
    QWidget *navigationTab = new QWidget();
    navigationTab->setStyleSheet("background-color: white; border-radius: 0 0 12px 12px;");

    // 使用垂直布局作为主布局
    QVBoxLayout *mainNavLayout = new QVBoxLayout(navigationTab);
    mainNavLayout->setSpacing(12);
    mainNavLayout->setContentsMargins(15, 15, 15, 15);

    // 导航状态
    QGroupBox *navStatusGroup = createStyledGroupBox("导航状态");
    QVBoxLayout *navStatusLayout = new QVBoxLayout(navStatusGroup);

    QLabel *navStatusLabel = new QLabel("⚪ 未就绪");
    navStatusLabel->setAlignment(Qt::AlignCenter);
    navStatusLabel->setStyleSheet(
        "QLabel {"
        "    background-color: #f5f5f5;"
        "    border-radius: 6px;"
        "    padding: 12px;"
        "    font-weight: bold;"
        "    font-size: 12px;"
        "    color: #757575;"
        "}"
    );
    navStatusLayout->addWidget(navStatusLabel);
    mainNavLayout->addWidget(navStatusGroup);

    // 使用水平布局放置地图控制、导航控制和一键返航（三列）
    QHBoxLayout *controlLayout = new QHBoxLayout();
    controlLayout->setSpacing(12);

    // 地图控制
    QGroupBox *mapControlGroup = createStyledGroupBox("地图控制");
    QVBoxLayout *mapControlLayout = new QVBoxLayout(mapControlGroup);

    QPushButton *loadMapButton = createStyledButton("🗺️ 载入地图", "#2196F3", "#1976D2", true);
    loadMapButton->setEnabled(false);

    QPushButton *closeMapButton = createStyledButton("❌ 关闭地图", "#f44336", "#d32f2f", true);
    closeMapButton->setEnabled(false);

    mapControlLayout->addWidget(loadMapButton);
    mapControlLayout->addWidget(closeMapButton);

    // 导航控制
    QGroupBox *navControlGroup = createStyledGroupBox("导航控制");
    QVBoxLayout *navControlLayout = new QVBoxLayout(navControlGroup);

    QPushButton *startNavButton = createStyledButton("▶️ 开始导航", "#4CAF50", "#388E3C", true);
    startNavButton->setEnabled(false);

    QPushButton *stopNavButton = createStyledButton("⏹️ 停止导航", "#f44336", "#d32f2f", true);
    stopNavButton->setEnabled(false);

    navControlLayout->addWidget(startNavButton);
    navControlLayout->addWidget(stopNavButton);

    // === 新增：一键返航按钮（导航过程中禁用） ===
    QGroupBox *returnGroup = createStyledGroupBox("返航控制");
    QVBoxLayout *returnLayout = new QVBoxLayout(returnGroup);

    QPushButton *returnHomeButton = createStyledButton("🏠 一键返航", "#9C27B0", "#7B1FA2", true);
    returnHomeButton->setEnabled(false);  // 初始状态禁用

    returnLayout->addWidget(returnHomeButton);
    // 添加一个空白占位保持高度一致
    QLabel *placeholderLabel = new QLabel("");
    placeholderLabel->setFixedHeight(36);
    returnLayout->addWidget(placeholderLabel);

    controlLayout->addWidget(mapControlGroup);
    controlLayout->addWidget(navControlGroup);
    controlLayout->addWidget(returnGroup);
    mainNavLayout->addLayout(controlLayout);

    // === 航点选择功能 ===
    QGroupBox *waypointGroup = createStyledGroupBox("航点选择");
    QVBoxLayout *waypointLayout = new QVBoxLayout(waypointGroup);

    // 航点按钮和清空按钮的水平布局
    QHBoxLayout *waypointTopLayout = new QHBoxLayout();
    waypointTopLayout->setSpacing(8);

    // 航点按钮容器（水平布局）
    QHBoxLayout *waypointButtonLayout = new QHBoxLayout();
    waypointButtonLayout->setSpacing(8);

    QPushButton *waypoint1Button = createStyledButton("📍 航点一", "#FF9800", "#F57C00", false);
    QPushButton *waypoint2Button = createStyledButton("📍 航点二", "#FF9800", "#F57C00", false);
    QPushButton *waypoint3Button = createStyledButton("📍 航点三", "#FF9800", "#F57C00", false);

    waypoint1Button->setEnabled(false);
    waypoint2Button->setEnabled(false);
    waypoint3Button->setEnabled(false);

    waypointButtonLayout->addWidget(waypoint1Button);
    waypointButtonLayout->addWidget(waypoint2Button);
    waypointButtonLayout->addWidget(waypoint3Button);

    // 清空按钮
    QPushButton *clearWaypointButton = createStyledButton("🗑️ 清空列表", "#f44336", "#d32f2f", false);
    clearWaypointButton->setEnabled(false);  // 初始状态禁用
    clearWaypointButton->setFixedWidth(100);  // 固定宽度

    waypointTopLayout->addLayout(waypointButtonLayout);
    waypointTopLayout->addWidget(clearWaypointButton);
    waypointTopLayout->addStretch();  // 右侧弹性空间

    // 航点列表窗格
    QLabel *waypointListLabel = new QLabel("已选航点列表：");
    waypointListLabel->setStyleSheet("font-weight: bold; color: #333; margin-top: 8px; font-size: 12px;");

    QListWidget *waypointListWidget = new QListWidget();
    waypointListWidget->setStyleSheet(
        "QListWidget {"
        "    background-color: #f8f9fa;"
        "    border: 2px solid #e0e0e0;"
        "    border-radius: 6px;"
        "    padding: 4px;"
        "    min-height: 80px;"
        "    max-height: 100px;"
        "    font-size: 11px;"
        "}"
        "QListWidget::item {"
        "    padding: 6px;"
        "    border-bottom: 1px solid #e0e0e0;"
        "}"
        "QListWidget::item:selected {"
        "    background-color: #e3f2fd;"
        "    color: #1976d2;"
        "}"
    );

    waypointLayout->addLayout(waypointTopLayout);
    waypointLayout->addWidget(waypointListLabel);
    waypointLayout->addWidget(waypointListWidget);

    mainNavLayout->addWidget(waypointGroup);
    mainNavLayout->addStretch();

    tabWidget->addTab(manualTab, "🎮 手动控制");
    tabWidget->addTab(navigationTab, "🤖 自主导航");

    leftLayout->addWidget(tabWidget);

    // === 右侧地图显示区域（缩小） ===
    QWidget *rightPanel = new QWidget();
    rightPanel->setStyleSheet("background-color: transparent;");
    QVBoxLayout *rightLayout = new QVBoxLayout(rightPanel);
    rightLayout->setSpacing(12);
    rightLayout->setContentsMargins(8, 0, 0, 0);

    // 地图显示卡片
    QWidget *mapCard = new QWidget();
    mapCard->setStyleSheet("background-color: white; border-radius: 12px;");
    applyShadow(mapCard, 12, 0, QColor(0, 0, 0, 15));

    QVBoxLayout *mapCardLayout = new QVBoxLayout(mapCard);
    mapCardLayout->setSpacing(12);
    mapCardLayout->setContentsMargins(15, 15, 15, 15);

    QHBoxLayout *mapTitleLayout = new QHBoxLayout();
    QLabel *mapIcon = new QLabel("🗺️");
    QLabel *mapTitle = new QLabel("地图与轨迹显示");
    mapTitle->setStyleSheet("font-size: 16px; font-weight: bold; color: #333;");
    mapTitleLayout->addWidget(mapIcon);
    mapTitleLayout->addWidget(mapTitle);
    mapTitleLayout->addStretch();

    // 地图显示控件
    MapDisplayWidget *mapDisplayWidget = new MapDisplayWidget();
    mapDisplayWidget->setMinimumSize(500, 450);
    mapDisplayWidget->setStyleSheet(
        "MapDisplayWidget {"
        "    border: 2px solid #e0e0e0;"
        "    border-radius: 8px;"
        "    background-color: #fafafa;"
        "}"
    );

    // ROS2桥接控制
//    QWidget *rosBridgeWidget = new QWidget();
//    rosBridgeWidget->setStyleSheet("background-color: #f8f9fa; border-radius: 8px;");

//    QHBoxLayout *rosBridgeLayout = new QHBoxLayout(rosBridgeWidget);
//    rosBridgeLayout->setSpacing(8);
//    rosBridgeLayout->setContentsMargins(8, 6, 8, 6);

//    QPushButton *startRosBridgeButton = createStyledButton("🔌 启动订阅", "#4CAF50", "#388E3C");
//    QPushButton *stopRosBridgeButton = createStyledButton("🔌 停止订阅", "#f44336", "#d32f2f");
//    stopRosBridgeButton->setEnabled(false);

//    QPushButton *testPosButton = createStyledButton("🧪 测试位置", "#9C27B0", "#7B1FA2");
//    QPushButton *manualPoseButton = createStyledButton("📍 手动设置", "#FF9800", "#F57C00");

//    rosBridgeLayout->addWidget(startRosBridgeButton);
//    rosBridgeLayout->addWidget(stopRosBridgeButton);
//    rosBridgeLayout->addWidget(testPosButton);
//    rosBridgeLayout->addWidget(manualPoseButton);

//    QLabel *rosBridgeStatusLabel = new QLabel("⚪ 未启动");
//    rosBridgeStatusLabel->setAlignment(Qt::AlignCenter);
//    rosBridgeStatusLabel->setStyleSheet(
//        "QLabel {"
//        "    background-color: #f5f5f5;"
//        "    border-radius: 4px;"
//        "    padding: 6px;"
//        "    font-size: 10px;"
//        "}"
//    );

    mapCardLayout->addLayout(mapTitleLayout);
    mapCardLayout->addWidget(mapDisplayWidget);
//    mapCardLayout->addWidget(rosBridgeWidget);
//    mapCardLayout->addWidget(rosBridgeStatusLabel);

    rightLayout->addWidget(mapCard);
    // === 摄像头监控卡片 ===
    QWidget *cameraCard = new QWidget();
    cameraCard->setStyleSheet("background-color: white; border-radius: 12px;");
    applyShadow(cameraCard, 8, 0, QColor(0, 0, 0, 12));

    QVBoxLayout *cameraCardLayout = new QVBoxLayout(cameraCard);
    cameraCardLayout->setSpacing(8);
    cameraCardLayout->setContentsMargins(12, 12, 12, 12);

    QHBoxLayout *cameraTitleLayout = new QHBoxLayout();
    QLabel *cameraIcon = new QLabel("📷");
    QLabel *cameraTitle = new QLabel("摄像头监控");
    cameraTitle->setStyleSheet("font-size: 14px; font-weight: bold; color: #333;");
    cameraTitleLayout->addWidget(cameraIcon);
    cameraTitleLayout->addWidget(cameraTitle);
    cameraTitleLayout->addStretch();

    // 摄像头显示控件
    CameraWidget *cameraWidget = new CameraWidget();
    cameraWidget->setMinimumHeight(240);
    cameraWidget->setStyleSheet(
        "CameraWidget {"
        "    border: 2px solid #e0e0e0;"
        "    border-radius: 8px;"
        "    background-color: #000;"
        "}"
    );

    // 摄像头控制栏
    QHBoxLayout *cameraControlLayout = new QHBoxLayout();
    cameraControlLayout->setSpacing(6);

    QLabel *fpsLabel = new QLabel("帧率: -- fps");
    fpsLabel->setStyleSheet("font-size: 10px; color: #666; background-color: #f5f5f5; padding: 4px 8px; border-radius: 4px;");

    QPushButton *startCameraBtn = createStyledButton("启动", "#4CAF50", "#388E3C");
    startCameraBtn->setFixedHeight(30);
    startCameraBtn->setFixedWidth(60);

    QPushButton *stopCameraBtn = createStyledButton("停止", "#f44336", "#d32f2f");
    stopCameraBtn->setFixedHeight(30);
    stopCameraBtn->setFixedWidth(60);
    stopCameraBtn->setEnabled(false);

    QComboBox *fpsCombo = new QComboBox();
    fpsCombo->addItems({"5 fps", "10 fps", "15 fps", "20 fps"});
    fpsCombo->setCurrentIndex(2);  // 默认15fps
    fpsCombo->setFixedWidth(70);
    fpsCombo->setStyleSheet("font-size: 10px;");

    QCheckBox *showFpsCheck = new QCheckBox("显示帧率");
    showFpsCheck->setChecked(true);
    showFpsCheck->setStyleSheet("font-size: 10px;");

    cameraControlLayout->addWidget(fpsLabel);
    cameraControlLayout->addStretch();
    cameraControlLayout->addWidget(startCameraBtn);
    cameraControlLayout->addWidget(stopCameraBtn);
    cameraControlLayout->addWidget(fpsCombo);
    cameraControlLayout->addWidget(showFpsCheck);

    cameraCardLayout->addLayout(cameraTitleLayout);
    cameraCardLayout->addWidget(cameraWidget);
    cameraCardLayout->addLayout(cameraControlLayout);

    rightLayout->addWidget(cameraCard);
    // 设置分割器
    mainSplitter->addWidget(leftPanel);
    mainSplitter->addWidget(rightPanel);
    mainSplitter->setStretchFactor(0, 4);
    mainSplitter->setStretchFactor(1, 6);
    mainSplitter->setSizes(QList<int>() << 550 << 650);

    mainLayout->addWidget(mainSplitter);

    // 状态栏（缩小）
    QWidget *statusBar = new QWidget();
    statusBar->setStyleSheet("background-color: white; border-radius: 8px;");
    applyShadow(statusBar, 4, 0, QColor(0, 0, 0, 8));

    QHBoxLayout *statusBarLayout = new QHBoxLayout(statusBar);
    statusBarLayout->setContentsMargins(15, 8, 15, 8);

    QLabel *systemStatusLabel = new QLabel("⚡ 系统状态: 正常");
    systemStatusLabel->setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 11px;");

    QLabel *timeLabel = new QLabel(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    timeLabel->setStyleSheet("color: #9e9e9e; font-size: 11px;");

    QTimer *timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, [timeLabel]() {
        timeLabel->setText(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    });
    timer->start(1000);

    statusBarLayout->addWidget(systemStatusLabel);
    statusBarLayout->addStretch();
    statusBarLayout->addWidget(timeLabel);

    mainLayout->addWidget(statusBar);

    // === 创建管理器 ===
    LaunchManager launchManager;
    ManualController manualController;
    RosMapBridge rosMapBridge;

    // === 地图显示设置 ===
    rosMapBridge.setMapTopic("/map");
    rosMapBridge.setRobotPoseTopic("/amcl_pose");
    rosMapBridge.setGoalPoseTopic("/goal_pose");
    rosMapBridge.setMapFrame("map");
    rosMapBridge.setRobotFrame("base_footprint");

    // 连接信号
//    QObject::connect(testPosButton, &QPushButton::clicked, [&]() {
//        static float testX = -4.0, testY = -1.0;
//        qDebug() << "手动设置机器人位置: (" << testX << ", " << testY << ")";
//        mapDisplayWidget->updateRobotPose(testX, testY, 0);
//        testX += 0.1;
//        testY += 0.1;
//    });

//    QObject::connect(manualPoseButton, &QPushButton::clicked, [&]() {
//        mapDisplayWidget->setTestRobotPosition(0, 0, 0);
//    });

    // ROS2地图桥接信号
    bool mapConnected = QObject::connect(&rosMapBridge, &RosMapBridge::mapReceived,
                                         mapDisplayWidget, &MapDisplayWidget::updateMap,
                                         Qt::QueuedConnection);

    bool poseConnected = QObject::connect(&rosMapBridge, &RosMapBridge::robotPoseUpdated,
                                          mapDisplayWidget, &MapDisplayWidget::updateRobotPose,
                                          Qt::QueuedConnection);

    QObject::connect(&rosMapBridge, &RosMapBridge::mapReceived,
                     [&](const QImage& img, float res, float ox, float oy) {
        mapStatusLabel->setText(QString("📡 地图已接收 (%1x%2)")
                                .arg(img.width()).arg(img.height()));
    });
    QObject::connect(&rosMapBridge, &RosMapBridge::batteryUpdated,
        [batteryLabel](float percent) {
            batteryLabel->setText(QString("%1%").arg(percent, 0, 'f', 1));
            // 根据电量改变颜色
            if (percent < 20) {
                batteryLabel->setStyleSheet(
                    "QLabel {"
                    "    color: #f44336;"
                    "    font-weight: bold;"
                    "    font-size: 12px;"
                    "    background-color: #ffebee;"
                    "    border-radius: 12px;"
                    "    padding: 4px 12px;"
                    "}"
                );
            } else if (percent < 50) {
                batteryLabel->setStyleSheet(
                    "QLabel {"
                    "    color: #ff9800;"
                    "    font-weight: bold;"
                    "    font-size: 12px;"
                    "    background-color: #fff3e0;"
                    "    border-radius: 12px;"
                    "    padding: 4px 12px;"
                    "}"
                );
            } else {
                batteryLabel->setStyleSheet(
                    "QLabel {"
                    "    color: #4caf50;"
                    "    font-weight: bold;"
                    "    font-size: 12px;"
                    "    background-color: #e8f5e9;"
                    "    border-radius: 12px;"
                    "    padding: 4px 12px;"
                    "}"
                );
            }
        }
    );
    QObject::connect(&rosMapBridge, &RosMapBridge::robotPoseUpdated,
                     [&](float x, float y, float theta) {
        robotPoseLabel->setText(QString("📍 位置: (%.2f, %.2f) 方向: %.1f°")
                                .arg(x).arg(y).arg(theta * 180 / M_PI));
    });

//    QObject::connect(&rosMapBridge, &RosMapBridge::statusChanged,
//                     [&](const QString& status) {
//        rosBridgeStatusLabel->setText(status);
//    });

//    QObject::connect(&rosMapBridge, &RosMapBridge::errorOccurred,
//                     [&](const QString& error) {
//        rosBridgeStatusLabel->setText("❌ 错误");
//        QMessageBox::warning(&window, "ROS2桥接错误", error);
//    });

//    // 启动ROS2桥接按钮
//    QObject::connect(startRosBridgeButton, &QPushButton::clicked, [&]() {
//        rosBridgeStatusLabel->setText("🔄 初始化中...");
//        startRosBridgeButton->setEnabled(false);

//        if (rosMapBridge.initROS2()) {
//            rosBridgeStatusLabel->setText("✅ 初始化成功");
//            stopRosBridgeButton->setEnabled(true);
//            rosMapBridge.startSubscriptions();
//        } else {
//            rosBridgeStatusLabel->setText("❌ 初始化失败");
//            startRosBridgeButton->setEnabled(true);
//            QMessageBox::warning(&window, "ROS2桥接失败", "无法初始化ROS2节点");
//        }
//    });

//    // 停止ROS2桥接按钮
//    QObject::connect(stopRosBridgeButton, &QPushButton::clicked, [&]() {
//        rosBridgeStatusLabel->setText("🔄 停止中...");
//        stopRosBridgeButton->setEnabled(false);

//        rosMapBridge.stopSubscriptions();
//        rosMapBridge.shutdownROS2();

//        rosBridgeStatusLabel->setText("⚪ 已停止");
//        startRosBridgeButton->setEnabled(true);
//    });
    // === 摄像头控制信号连接 ===
    // 设置摄像头URL（修改为你的ESP32-CAM实际IP）
    cameraWidget->setCameraUrl("<CAMERA_IP>", 80);

    // 启动摄像头
    QObject::connect(startCameraBtn, &QPushButton::clicked, [&]() {
        int fps = 15;
        switch(fpsCombo->currentIndex()) {
            case 0: fps = 5; break;
            case 1: fps = 10; break;
            case 2: fps = 15; break;
            case 3: fps = 20; break;
        }
        cameraWidget->startStream(fps);
        startCameraBtn->setEnabled(false);
        stopCameraBtn->setEnabled(true);
    });

    // 停止摄像头
    QObject::connect(stopCameraBtn, &QPushButton::clicked, [&]() {
        cameraWidget->stopStream();
        startCameraBtn->setEnabled(true);
        stopCameraBtn->setEnabled(false);
    });

    // 显示帧率
    QObject::connect(showFpsCheck, &QCheckBox::toggled, [&](bool checked) {
        cameraWidget->showFps(checked);
    });

    // 更新帧率显示
    QObject::connect(cameraWidget, &CameraWidget::fpsUpdated,
                     [fpsLabel](double fps) {
        fpsLabel->setText(QString("帧率: %1 fps").arg(fps, 0, 'f', 1));
    });
    // === 连接控制逻辑 ===
    QObject::connect(connectButton, &QPushButton::clicked, [&]() {
        if (!launchManager.isConnected()) {
            statusLabel->setText("🟡 连接中...");
            connectButton->setEnabled(false);
            launchManager.forceStopAllProcesses();

            QTimer::singleShot(500, [&]() {
                if (launchManager.startRobotConnection()) {
                    connectButton->setText("断开连接");
                    statusLabel->setText("🟢 已连接");
                    statusLabel->setStyleSheet(
                        "QLabel {"
                        "    background-color: #d4edda;"
                        "    border-radius: 6px;"
                        "    padding: 10px;"
                        "    font-weight: bold;"
                        "    color: #155724;"
                        "    font-size: 12px;"
                        "}"
                    );

                    // 连接成功后启用所有相关按钮
                    loadMapButton->setEnabled(true);
                    startNavButton->setEnabled(true);
                    stopNavButton->setEnabled(false);
                    returnHomeButton->setEnabled(true);  // 连接后启用一键返航
//                    startRosBridgeButton->setEnabled(true);

                    // 启用航点按钮
                    waypoint1Button->setEnabled(true);
                    waypoint2Button->setEnabled(true);
                    waypoint3Button->setEnabled(true);

                    for (auto btn : directionButtons) btn->setEnabled(true);
                    increaseSpeedButton->setEnabled(true);
                    decreaseSpeedButton->setEnabled(true);
                    resetSpeedButton->setEnabled(true);

                    // 重要：初始化手动控制器
                    manualController.initROS2();
                } else {
                    connectButton->setEnabled(true);
                    statusLabel->setText("🔴 连接失败");
                    QMessageBox::warning(&window, "连接失败", "无法启动机器人连接");
                }
            });
        } else {
            statusLabel->setText("🟡 断开中...");
            connectButton->setEnabled(false);

            launchManager.stopRobotConnection();
            rosMapBridge.shutdownROS2();

            QTimer::singleShot(1500, [&]() {
                launchManager.forceStopAllProcesses();
                connectButton->setText("连接机器人");
                statusLabel->setText("⚪ 未连接");
                cameraWidget->stopStream();
                startCameraBtn->setEnabled(false);
                stopCameraBtn->setEnabled(false);
                statusLabel->setStyleSheet(
                    "QLabel {"
                    "    background-color: #f5f5f5;"
                    "    border-radius: 6px;"
                    "    padding: 10px;"
                    "    font-weight: bold;"
                    "    color: #757575;"
                    "    font-size: 12px;"
                    "}"
                );
                connectButton->setEnabled(true);

                // 断开后禁用所有控制按钮
                for (auto btn : directionButtons) btn->setEnabled(false);
                increaseSpeedButton->setEnabled(false);
                decreaseSpeedButton->setEnabled(false);
                resetSpeedButton->setEnabled(false);

                loadMapButton->setEnabled(false);
                closeMapButton->setEnabled(false);
                startNavButton->setEnabled(false);
                stopNavButton->setEnabled(false);
                returnHomeButton->setEnabled(false);  // 断开后禁用

//                startRosBridgeButton->setEnabled(false);
//                stopRosBridgeButton->setEnabled(false);

                // 禁用航点按钮
                waypoint1Button->setEnabled(false);
                waypoint2Button->setEnabled(false);
                waypoint3Button->setEnabled(false);
                clearWaypointButton->setEnabled(false);
                // 清空航点列表
                waypointListWidget->clear();

                navStatusLabel->setText("⚪ 未就绪");
                mapStatusLabel->setText("📡 等待地图数据...");
                robotPoseLabel->setText("📍 位置: 未知");
//                rosBridgeStatusLabel->setText("⚪ 未启动");

                mapDisplayWidget->clearTrajectory();
                mapDisplayWidget->resetView();
            });
        }
    });

    // 清理按钮
    QObject::connect(cleanupButton, &QPushButton::clicked, [&]() {
        // 显示清理中状态
        statusLabel->setText("🟡 清理中...");
        connectButton->setEnabled(false);
        cleanupButton->setEnabled(false);
//        startRosBridgeButton->setEnabled(false);
//        stopRosBridgeButton->setEnabled(false);

        qDebug() << "=== 开始彻底清理 ===";

        // === 新增：先清理特定ROS2节点 ===
        qDebug() << "清理ROS2节点: odom2tf, robot_state_publisher...";
        system("pkill -9 -f 'odom2tf' 2>/dev/null");
        system("pkill -9 -f 'robot_state_publisher' 2>/dev/null");
        QThread::msleep(200);  // 等待进程被杀死

        // 1. 清理LaunchManager的所有进程
        launchManager.forceStopAllProcesses();
        qDebug() << "LaunchManager进程清理完成";

        // 2. 强制清理ROS2桥接资源
//        rosMapBridge.forceCleanup();
        qDebug() << "ROS2桥接清理完成";

        // 3. 停止手动控制器
        manualController.shutdownROS2();
        qDebug() << "手动控制器清理完成";

        // 4. 额外清理：杀掉所有残留的ROS2相关进程
        system("pkill -9 -f 'ros2' 2>/dev/null");
        system("pkill -9 -f 'rclcpp' 2>/dev/null");
        system("pkill -9 -f 'qt_map_bridge' 2>/dev/null");
        system("pkill -9 -f 'python3.*ros' 2>/dev/null");

        // 延迟确保清理完成
        QTimer::singleShot(1500, [&]() {
            statusLabel->setText("⚪ 未连接");
            cameraWidget->stopStream();
            startCameraBtn->setEnabled(false);
            stopCameraBtn->setEnabled(false);
            statusLabel->setStyleSheet(
                "QLabel {"
                "    background-color: #f5f5f5;"
                "    border-radius: 6px;"
                "    padding: 10px;"
                "    font-weight: bold;"
                "    color: #757575;"
                "    font-size: 12px;"
                "}"
            );
            connectButton->setText("连接机器人");
            connectButton->setEnabled(true);
            cleanupButton->setEnabled(true);
//            startRosBridgeButton->setEnabled(true);

            // 断开后禁用所有控制按钮
            for (auto btn : directionButtons) btn->setEnabled(false);
            increaseSpeedButton->setEnabled(false);
            decreaseSpeedButton->setEnabled(false);
            resetSpeedButton->setEnabled(false);

            loadMapButton->setEnabled(false);
            closeMapButton->setEnabled(false);
            startNavButton->setEnabled(false);
            stopNavButton->setEnabled(false);
            returnHomeButton->setEnabled(false);

            // 禁用航点按钮并清空列表
            waypoint1Button->setEnabled(false);
            waypoint2Button->setEnabled(false);
            waypoint3Button->setEnabled(false);
            waypointListWidget->clear();

            navStatusLabel->setText("⚪ 未就绪");
            mapStatusLabel->setText("📡 等待地图数据...");
            robotPoseLabel->setText("📍 位置: 未知");
//            rosBridgeStatusLabel->setText("⚪ 未启动");

            mapDisplayWidget->clearTrajectory();
            mapDisplayWidget->resetView();

            qDebug() << "=== 清理完成 ===";
            QMessageBox::information(&window, "清理完成", "所有进程和ROS2节点已清理");
        });
    });
    // === 航点按钮信号连接 ===
    QObject::connect(waypoint1Button, &QPushButton::clicked, [&]() {
        bool exists = false;
        for (int i = 0; i < waypointListWidget->count(); ++i) {
            if (waypointListWidget->item(i)->text() == "航点一") {
                exists = true;
                break;
            }
        }
        if (!exists) {
            waypointListWidget->addItem("航点一");
            waypoint1Button->setEnabled(false);
            clearWaypointButton->setEnabled(true);
        }
    });

    QObject::connect(waypoint2Button, &QPushButton::clicked, [&]() {
        bool exists = false;
        for (int i = 0; i < waypointListWidget->count(); ++i) {
            if (waypointListWidget->item(i)->text() == "航点二") {
                exists = true;
                break;
            }
        }
        if (!exists) {
            waypointListWidget->addItem("航点二");
            waypoint2Button->setEnabled(false);
            clearWaypointButton->setEnabled(true);
        }
    });

    QObject::connect(waypoint3Button, &QPushButton::clicked, [&]() {
        bool exists = false;
        for (int i = 0; i < waypointListWidget->count(); ++i) {
            if (waypointListWidget->item(i)->text() == "航点三") {
                exists = true;
                break;
            }
        }
        if (!exists) {
            waypointListWidget->addItem("航点三");
            waypoint3Button->setEnabled(false);
            clearWaypointButton->setEnabled(true);
        }
    });
    // === 航点清空按钮信号连接 ===
       QObject::connect(clearWaypointButton, &QPushButton::clicked, [&]() {
           // 清空航点列表
           waypointListWidget->clear();

           // 重新启用所有航点按钮
           waypoint1Button->setEnabled(true);
           waypoint2Button->setEnabled(true);
           waypoint3Button->setEnabled(true);

           // 禁用清空按钮（列表为空时禁用）
           clearWaypointButton->setEnabled(false);

           // 可选：显示提示消息
           navStatusLabel->setText("🟢 航点已清空");
           navStatusLabel->setStyleSheet(
               "QLabel {"
               "    background-color: #d4edda;"
               "    border-radius: 6px;"
               "    padding: 12px;"
               "    font-weight: bold;"
               "    color: #155724;"
               "    font-size: 12px;"
               "}"
           );

           // 2秒后恢复状态（如果导航未在进行）
           QTimer::singleShot(2000, [&]() {
               if (!launchManager.isNavigationRunning() && !launchManager.isNavigationQueueRunning()) {
                   navStatusLabel->setText("🟢 地图已载入");
                   navStatusLabel->setStyleSheet(
                       "QLabel {"
                       "    background-color: #d4edda;"
                       "    border-radius: 6px;"
                       "    padding: 12px;"
                       "    font-weight: bold;"
                       "    color: #155724;"
                       "    font-size: 12px;"
                       "}"
                   );
               }
           });
       });
    // Launch管理器信号
    QObject::connect(&launchManager, &LaunchManager::connectionStarted, [&]() {
        statusLabel->setText("🟢 已连接");
        connectButton->setEnabled(true);
    });

    QObject::connect(&launchManager, &LaunchManager::connectionStopped, [&]() {
        statusLabel->setText("⚪ 未连接");
        connectButton->setText("连接机器人");
        connectButton->setEnabled(true);
    });

    QObject::connect(&launchManager, &LaunchManager::connectionError, [&](const QString& error) {
        statusLabel->setText("🔴 错误");
        QMessageBox::critical(&window, "连接错误", error);
        connectButton->setEnabled(true);
    });

    // === 手动控制信号槽（完全保持不变） ===
    QObject::connect(btnUpLeft, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_FORWARD_LEFT);
    });
    QObject::connect(btnUp, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_FORWARD);
    });
    QObject::connect(btnUpRight, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_FORWARD_RIGHT);
    });
    QObject::connect(btnLeft, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_LEFT);
    });
    QObject::connect(btnStop, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_STOP);
    });
    QObject::connect(btnRight, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_RIGHT);
    });
    QObject::connect(btnDownLeft, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_BACKWARD_LEFT);
    });
    QObject::connect(btnDown, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_BACKWARD);
    });
    QObject::connect(btnDownRight, &QPushButton::clicked, [&]() {
        manualController.sendCommand(ManualController::CMD_BACKWARD_RIGHT);
    });

    // 速度控制按钮
    QObject::connect(increaseSpeedButton, &QPushButton::clicked, [&]() {
        manualController.increaseSpeed();
        speedDisplay->setText(QString("速度: 线速 %1 m/s | 角速 %2 rad/s")
                            .arg(manualController.getLinearSpeed(), 0, 'f', 2)
                            .arg(manualController.getAngularSpeed(), 0, 'f', 2));
    });
    QObject::connect(decreaseSpeedButton, &QPushButton::clicked, [&]() {
        manualController.decreaseSpeed();
        speedDisplay->setText(QString("速度: 线速 %1 m/s | 角速 %2 rad/s")
                            .arg(manualController.getLinearSpeed(), 0, 'f', 2)
                            .arg(manualController.getAngularSpeed(), 0, 'f', 2));
    });
    QObject::connect(resetSpeedButton, &QPushButton::clicked, [&]() {
        manualController.resetSpeed();
        speedDisplay->setText(QString("速度: 线速 %1 m/s | 角速 %2 rad/s")
                            .arg(manualController.getLinearSpeed(), 0, 'f', 2)
                            .arg(manualController.getAngularSpeed(), 0, 'f', 2));
    });

    // 连接速度变化信号
    QObject::connect(&manualController, &ManualController::speedChanged,
                     [&](double linear, double angular) {
        speedDisplay->setText(QString("速度: 线速 %1 m/s | 角速 %2 rad/s")
                            .arg(linear, 0, 'f', 2)
                            .arg(angular, 0, 'f', 2));
    });

    // === 导航控制信号槽 ===
    QObject::connect(loadMapButton, &QPushButton::clicked, [&]() {
        navStatusLabel->setText("🟡 载入地图中...");
        navStatusLabel->setStyleSheet(
            "QLabel {"
            "    background-color: #fff3cd;"
            "    border-radius: 6px;"
            "    padding: 12px;"
            "    font-weight: bold;"
            "    color: #856404;"
            "    font-size: 12px;"
            "}"
        );
        loadMapButton->setEnabled(false);

        if (launchManager.startMapAndRVIZ()) {
            // 状态将在mapLoaded信号中更新
        } else {
            QTimer::singleShot(1000, [&]() {
                navStatusLabel->setText("🔴 载入失败");
                navStatusLabel->setStyleSheet(
                    "QLabel {"
                    "    background-color: #f8d7da;"
                    "    border-radius: 6px;"
                    "    padding: 12px;"
                    "    font-weight: bold;"
                    "    color: #721c24;"
                    "    font-size: 12px;"
                    "}"
                );
                loadMapButton->setEnabled(true);
            });
        }
    });

    QObject::connect(&launchManager, &LaunchManager::mapLoaded, [&](bool success, const QString& message) {
        if (success) {
//            navStatusLabel->setText("🟢 地图已载入");
//            navStatusLabel->setStyleSheet(
//                "QLabel {"
//                "    background-color: #d4edda;"
//                "    border-radius: 6px;"
//                "    padding: 12px;"
//                "    font-weight: bold;"
//                "    color: #155724;"
//                "    font-size: 12px;"
//                "}"
//            );
//            loadMapButton->setEnabled(false);
            closeMapButton->setEnabled(true);
            startNavButton->setEnabled(true);
            returnHomeButton->setEnabled(true);  // 地图加载后启用（导航还没开始）
            QMetaObject::Connection renderConn;
                    renderConn = QObject::connect(mapDisplayWidget, &MapDisplayWidget::mapRendered,
                        [&, renderConn]() mutable {  // 捕获 renderConn 的副本
                            navStatusLabel->setText("🟢 地图已载入");
                            navStatusLabel->setStyleSheet(
                                "QLabel {"
                                "    background-color: #d4edda;"
                                "    border-radius: 6px;"
                                "    padding: 12px;"
                                "    font-weight: bold;"
                                "    color: #155724;"
                                "    font-size: 12px;"
                                "}"
                            );
                            loadMapButton->setEnabled(false);
                            QObject::disconnect(renderConn);
                        });

                    // 超时保护
                    QTimer::singleShot(5000, [&, renderConn]() mutable {  // 捕获 renderConn 的副本
                        if (loadMapButton->isEnabled()) {
                            navStatusLabel->setText("🟢 地图已载入");
                            navStatusLabel->setStyleSheet(
                                "QLabel {"
                                "    background-color: #d4edda;"
                                "    border-radius: 6px;"
                                "    padding: 12px;"
                                "    font-weight: bold;"
                                "    color: #155724;"
                                "    font-size: 12px;"
                                "}"
                            );
                            loadMapButton->setEnabled(false);
                            QObject::disconnect(renderConn);
                        }
                    });
            if (!rosMapBridge.isInitialized()) {
                        QTimer::singleShot(1000, [&]() {  // 等待1秒
                            qDebug() << "地图加载完成，延迟初始化ROS2桥接...";
                            if (rosMapBridge.initROS2()) {
                                // 再延迟500ms启动订阅，确保节点稳定
                                QTimer::singleShot(500, [&]() {
                                    rosMapBridge.startSubscriptions();
                                    // 再延迟1秒主动请求地图
                                    QTimer::singleShot(1000, [&]() {
                                        rosMapBridge.requestMap();  // 主动请求地图
                                    });
                                });
                            }
                        });
             }else {
                     // 已初始化，直接请求地图
                     QTimer::singleShot(500, [&]() {
                     rosMapBridge.requestMap();
                     });
                   }
        } else {
            navStatusLabel->setText("🔴 载入失败");
            navStatusLabel->setStyleSheet(
                "QLabel {"
                "    background-color: #f8d7da;"
                "    border-radius: 6px;"
                "    padding: 12px;"
                "    font-weight: bold;"
                "    color: #721c24;"
                "    font-size: 12px;"
                "}"
            );
            loadMapButton->setEnabled(true);
            QMessageBox::warning(&window, "地图载入失败", message);
        }
    });

    QObject::connect(closeMapButton, &QPushButton::clicked, [&]() {
        navStatusLabel->setText("🟡 关闭地图中...");
        navStatusLabel->setStyleSheet(
            "QLabel {"
            "    background-color: #fff3cd;"
            "    border-radius: 6px;"
            "    padding: 12px;"
            "    font-weight: bold;"
            "    color: #856404;"
            "    font-size: 12px;"
            "}"
        );
        closeMapButton->setEnabled(false);
        // ===== 新增：先清空地图显示 =====
        mapDisplayWidget->clearMap();
        launchManager.stopMapAndRVIZ();
    });

    QObject::connect(&launchManager, &LaunchManager::mapStopped, [&]() {
        navStatusLabel->setText("⚪ 未就绪");
        navStatusLabel->setStyleSheet(
            "QLabel {"
            "    background-color: #f5f5f5;"
            "    border-radius: 6px;"
            "    padding: 12px;"
            "    font-weight: bold;"
            "    color: #757575;"
            "    font-size: 12px;"
            "}"
        );
        loadMapButton->setEnabled(true);
        closeMapButton->setEnabled(false);
        startNavButton->setEnabled(false);
        stopNavButton->setEnabled(false);
        returnHomeButton->setEnabled(false);  // 地图关闭后禁用

        rosMapBridge.stopSubscriptions();
    });

    // === 修改：开始导航按钮 - 实现航点队列导航 ===
    QObject::connect(startNavButton, &QPushButton::clicked, [&]() {
        // 检查是否有选中的航点
        if (waypointListWidget->count() == 0) {
            navStatusLabel->setText("🔴 请先选择航点");
            navStatusLabel->setStyleSheet(
                "QLabel {"
                "    background-color: #f8d7da;"
                "    border-radius: 6px;"
                "    padding: 12px;"
                "    font-weight: bold;"
                "    color: #721c24;"
                "    font-size: 12px;"
                "}"
            );
            QTimer::singleShot(2000, [&]() {
                if (!launchManager.isNavigationQueueRunning()) {
                    navStatusLabel->setText("🟢 地图已载入");
                    navStatusLabel->setStyleSheet(
                        "QLabel {"
                        "    background-color: #d4edda;"
                        "    border-radius: 6px;"
                        "    padding: 12px;"
                        "    font-weight: bold;"
                        "    color: #155724;"
                        "    font-size: 12px;"
                        "}"
                    );
                }
            });
            return;
        }

        navStatusLabel->setText("🟡 开始导航...");
        navStatusLabel->setStyleSheet(
            "QLabel {"
            "    background-color: #fff3cd;"
            "    border-radius: 6px;"
            "    padding: 12px;"
            "    font-weight: bold;"
            "    color: #856404;"
            "    font-size: 12px;"
            "}"
        );
        startNavButton->setEnabled(false);

        // 根据航点列表构建命令队列
        QStringList commands;
        for (int i = 0; i < waypointListWidget->count(); ++i) {
            QString waypointName = waypointListWidget->item(i)->text();
            if (waypointName == "航点一") {
                commands << "ros2 launch autopatrol_robot autopatrol.launch.py target_indices:=\"2\"";
            } else if (waypointName == "航点二") {
                commands << "ros2 launch autopatrol_robot autopatrol.launch.py target_indices:=\"3\"";
            } else if (waypointName == "航点三") {
                commands << "ros2 launch autopatrol_robot autopatrol.launch.py target_indices:=\"4\"";
            }
        }
        commands << "ros2 run fishbot_application nav_to_pose";
        // 启动队列导航
        if (launchManager.startNavigationWithQueue(commands)) {
            // 状态将在 navigationStarted 信号中更新
        } else {
            QTimer::singleShot(1000, [&]() {
                navStatusLabel->setText("🔴 启动失败");
                navStatusLabel->setStyleSheet(
                    "QLabel {"
                    "    background-color: #f8d7da;"
                    "    border-radius: 6px;"
                    "    padding: 12px;"
                    "    font-weight: bold;"
                    "    color: #721c24;"
                    "    font-size: 12px;"
                    "}"
                );
                startNavButton->setEnabled(true);
            });
        }
    });

    // === 修改：导航启动信号处理 - 支持队列导航消息 ===
    QObject::connect(&launchManager, &LaunchManager::navigationStarted, [&](bool success, const QString& message) {
        if (success) {
            if (message.contains("所有航点导航完成")) {
                // 所有航点完成
                navStatusLabel->setText("🟢 导航完成");
                navStatusLabel->setStyleSheet(
                    "QLabel {"
                    "    background-color: #d4edda;"
                    "    border-radius: 6px;"
                    "    padding: 12px;"
                    "    font-weight: bold;"
                    "    color: #155724;"
                    "    font-size: 12px;"
                    "}"
                );
                startNavButton->setEnabled(true);
                stopNavButton->setEnabled(false);
                returnHomeButton->setEnabled(true);

                // 清空航点列表并重置按钮
                waypointListWidget->clear();
                waypoint1Button->setEnabled(true);
                waypoint2Button->setEnabled(true);
                waypoint3Button->setEnabled(true);
                clearWaypointButton->setEnabled(false);
                if (launchManager.isConnected()) {
                    for (auto btn : directionButtons) btn->setEnabled(true);
                }
            } else if (message.contains("正在执行")) {
                // 正在执行某个航点
                navStatusLabel->setText(QString("🟢 %1").arg(message));
                navStatusLabel->setStyleSheet(
                    "QLabel {"
                    "    background-color: #d4edda;"
                    "    border-radius: 6px;"
                    "    padding: 12px;"
                    "    font-weight: bold;"
                    "    color: #155724;"
                    "    font-size: 12px;"
                    "}"
                );
                stopNavButton->setEnabled(true);
                returnHomeButton->setEnabled(false);

                // 导航进行中，禁用航点按钮和手动控制
                waypoint1Button->setEnabled(false);
                waypoint2Button->setEnabled(false);
                waypoint3Button->setEnabled(false);
                for (auto btn : directionButtons) btn->setEnabled(false);
            } else {
                // 兼容原有的导航启动消息
                navStatusLabel->setText("🟢 导航进行中");
                navStatusLabel->setStyleSheet(
                    "QLabel {"
                    "    background-color: #d4edda;"
                    "    border-radius: 6px;"
                    "    padding: 12px;"
                    "    font-weight: bold;"
                    "    color: #155724;"
                    "    font-size: 12px;"
                    "}"
                );
                stopNavButton->setEnabled(true);
                returnHomeButton->setEnabled(false);

                waypoint1Button->setEnabled(false);
                waypoint2Button->setEnabled(false);
                waypoint3Button->setEnabled(false);
                for (auto btn : directionButtons) btn->setEnabled(false);
            }
        } else {
            navStatusLabel->setText("🔴 启动失败");
            navStatusLabel->setStyleSheet(
                "QLabel {"
                "    background-color: #f8d7da;"
                "    border-radius: 6px;"
                "    padding: 12px;"
                "    font-weight: bold;"
                "    color: #721c24;"
                "    font-size: 12px;"
                "}"
            );
            startNavButton->setEnabled(true);
            QMessageBox::warning(&window, "导航启动失败", message);
        }
    });

    // === 修改：停止导航按钮 - 同时停止队列 ===
    QObject::connect(stopNavButton, &QPushButton::clicked, [&]() {
        navStatusLabel->setText("🟡 停止中...");
        navStatusLabel->setStyleSheet(
            "QLabel {"
            "    background-color: #fff3cd;"
            "    border-radius: 6px;"
            "    padding: 12px;"
            "    font-weight: bold;"
            "    color: #856404;"
            "    font-size: 12px;"
            "}"
        );
        stopNavButton->setEnabled(false);

        launchManager.stopAutonomousNavigation();
    });

    QObject::connect(&launchManager, &LaunchManager::navigationStopped, [&]() {
        navStatusLabel->setText("🟢 地图已载入");
        navStatusLabel->setStyleSheet(
            "QLabel {"
            "    background-color: #d4edda;"
            "    border-radius: 6px;"
            "    padding: 12px;"
            "    font-weight: bold;"
            "    color: #155724;"
            "    font-size: 12px;"
            "}"
        );
        startNavButton->setEnabled(true);
        stopNavButton->setEnabled(false);
        returnHomeButton->setEnabled(true);  // 导航停止后，重新启用一键返航

        // 导航结束后，重新启用未添加的航点按钮
        bool w1Exists = false, w2Exists = false, w3Exists = false;
        for (int i = 0; i < waypointListWidget->count(); ++i) {
            QString text = waypointListWidget->item(i)->text();
            if (text == "航点一") w1Exists = true;
            if (text == "航点二") w2Exists = true;
            if (text == "航点三") w3Exists = true;
        }
        waypoint1Button->setEnabled(!w1Exists && launchManager.isConnected());
        waypoint2Button->setEnabled(!w2Exists && launchManager.isConnected());
        waypoint3Button->setEnabled(!w3Exists && launchManager.isConnected());

        if (launchManager.isConnected()) {
            for (auto btn : directionButtons) btn->setEnabled(true);
        }
    });

    // === 一键返航按钮信号连接 ===
    QObject::connect(returnHomeButton, &QPushButton::clicked, [&]() {
        // 检查是否已连接并已加载地图
        if (!launchManager.isConnected()) {
            QMessageBox::warning(&window, "无法返航", "请先连接机器人");
            return;
        }

        if (!launchManager.isMapLoaded()) {
            QMessageBox::warning(&window, "无法返航", "请先加载地图");
            return;
        }

        // 如果导航正在进行中，先停止当前导航
        if (launchManager.isNavigationRunning() || launchManager.isNavigationQueueRunning()) {
            navStatusLabel->setText("🟡 停止当前导航，准备返航...");
            navStatusLabel->setStyleSheet(
                "QLabel {"
                "    background-color: #fff3cd;"
                "    border-radius: 6px;"
                "    padding: 12px;"
                "    font-weight: bold;"
                "    color: #856404;"
                "    font-size: 12px;"
                "}"
            );

            // 停止当前导航
            launchManager.stopAutonomousNavigation();

            // 延迟500ms后执行返航命令，确保导航进程完全停止
            QTimer::singleShot(500, [&]() {
                navStatusLabel->setText("🟡 执行一键返航...");
                navStatusLabel->setStyleSheet(
                    "QLabel {"
                    "    background-color: #fff3cd;"
                    "    border-radius: 6px;"
                    "    padding: 12px;"
                    "    font-weight: bold;"
                    "    color: #856404;"
                    "    font-size: 12px;"
                    "}"
                );
                returnHomeButton->setEnabled(false);
                startNavButton->setEnabled(false);

                // 构建返航命令
                QStringList commands;
                commands << "ros2 run fishbot_application nav_to_pose";

                // 启动返航导航
                if (launchManager.startNavigationWithQueue(commands)) {
                    // 状态将在 navigationStarted 信号中更新
                } else {
                    navStatusLabel->setText("🔴 返航启动失败");
                    navStatusLabel->setStyleSheet(
                        "QLabel {"
                        "    background-color: #f8d7da;"
                        "    border-radius: 6px;"
                        "    padding: 12px;"
                        "    font-weight: bold;"
                        "    color: #721c24;"
                        "    font-size: 12px;"
                        "}"
                    );
                    returnHomeButton->setEnabled(true);
                    startNavButton->setEnabled(true);
                    QMessageBox::warning(&window, "返航失败", "无法启动返航命令");
                }
            });
        } else {
            // 没有正在进行的导航，直接执行返航
            navStatusLabel->setText("🟡 执行一键返航...");
            navStatusLabel->setStyleSheet(
                "QLabel {"
                "    background-color: #fff3cd;"
                "    border-radius: 6px;"
                "    padding: 12px;"
                "    font-weight: bold;"
                "    color: #856404;"
                "    font-size: 12px;"
                "}"
            );
            returnHomeButton->setEnabled(false);
            startNavButton->setEnabled(false);

            // 构建返航命令
            QStringList commands;
            commands << "ros2 run fishbot_application nav_to_pose";

            // 启动返航导航
            if (launchManager.startNavigationWithQueue(commands)) {
                // 状态将在 navigationStarted 信号中更新
            } else {
                navStatusLabel->setText("🔴 返航启动失败");
                navStatusLabel->setStyleSheet(
                    "QLabel {"
                    "    background-color: #f8d7da;"
                    "    border-radius: 6px;"
                    "    padding: 12px;"
                    "    font-weight: bold;"
                    "    color: #721c24;"
                    "    font-size: 12px;"
                    "}"
                );
                returnHomeButton->setEnabled(true);
                startNavButton->setEnabled(true);
                QMessageBox::warning(&window, "返航失败", "无法启动返航命令");
            }
        }
    });

    window.show();
    return app.exec();
}
