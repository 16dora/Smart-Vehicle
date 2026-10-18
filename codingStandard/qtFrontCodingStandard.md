## 十二、Smart Vehicle 前端 Qt UI 设计规范

本章适用于 Smart Vehicle 配套工具的 Qt 前端界面，包括主窗口、功能页面、参数面板、状态面板、图像显示窗口、日志窗口以及与后端服务交互的 UI 层。

### 12.1 基本原则

1. **UI 只负责显示与交互**：UI 层不得直接调用硬件 SDK、算法 SDK 或耗时业务逻辑。
2. **界面状态必须可追溯**：设备连接、任务执行、参数变更、错误状态必须在界面和日志中都有明确体现。
3. **布局优先于固定坐标**：除特殊图标和状态灯外，禁止依赖绝对坐标设计界面。
4. **控件命名必须稳定**：`objectName` 一旦被代码引用，不得随意修改。
5. **禁止 UI 线程阻塞**：耗时操作必须放到子线程中执行。

---

### 12.2 UI 文件与类命名

#### 12.2.1 UI 文件命名

- **格式**：全小写，下划线分隔。
- **扩展名**：`.ui`。
- **原则**：文件名描述页面用途，不使用无意义编号。

```text
// [推荐]
main_window.ui
camera_control_panel.ui
device_param_page.ui
image_viewer_widget.ui

// [禁止]
Form.ui
Widget1.ui
new_widget.ui
```

#### 12.2.2 Qt 类命名

- **类名**：使用 PascalCase。
- **页面类**：建议以 `page` / `panel` / `widget` / `dialog` / `window` 结尾。
- **文件名**：仍然保持小写下划线。

```cpp
camera_control_panel.h
};
```

---

### 12.3 UI 控件命名规则

#### 12.3.1 总体规则

- 控件 `objectName` 统一使用：`控件前缀_功能名`。
- 功能名推荐使用小驼峰，例如：`btn_openCamera`、`label_cameraTemperature`。
- 已经被代码引用的旧控件名不建议随意修改，避免破坏自动连接槽函数。
- 控件名必须表达业务含义，禁止使用 `pushButton_1`、`label_2`、`widget_3` 这类默认名称。
- **固定显示且不会在代码中访问的静态 Label** 可以使用默认命名。
- **动态显示、信号槽连接、代码访问、状态更新相关控件必须规范命名**。
- [禁止] **同名控件**：AI 辅助生成 `.ui` 时尤其容易产生重复 `objectName`，会导致 UI 文件异常或代码连接错误。

#### 12.3.2 常用控件命名前缀

| 控件类型 | 前缀 | 示例 | 使用说明 |
|---|---:|---|---|
| `QPushButton` | `btn_` | `btn_openCamera` | 普通按钮、执行按钮、切换按钮 |
| `QToolButton` | `tbtn_` | `tbtn_selectRoi` | 工具栏按钮、小图标按钮 |
| `QLabel` 动态状态 | `label_` | `label_cameraTemperature` | 温度、状态、错误码等 |
| `QLineEdit` | `le_` | `le_lotId` | 文本输入 |
| `QTextEdit` | `te_` | `te_comment` | 多行富文本输入 |
| `QPlainTextEdit` | `pt_` | `pt_log` | 日志、纯文本输出 |
| `QSpinBox` | `sbox_` | `sbox_retryCount` | 整数输入 |
| `QDoubleSpinBox` | `dsbox_` | `dsbox_exposureTimeMs` | 浮点数输入，必须带单位含义 |
| `QComboBox` | `cbox_` | `cbox_cameraSource` | 枚举选择 |
| `QCheckBox` | `check_` | `check_enableAutoSave` | 独立开关项 |
| `QRadioButton` | `rbtn_` | `rbtn_localSource` | 互斥选项 |
| `QTableWidget` | `table_` | `table_resultList` | 小数据量表格 |
| `QTableView` | `tableView_` | `tableView_resultList` | 大数据量表格，推荐 Model/View |
| `QTreeWidget` | `tree_` | `tree_recipeItems` | 小数据量树 |
| `QTreeView` | `treeView_` | `treeView_fileModel` | 大数据量树，推荐 Model/View |
| `QGraphicsView` | `gview_` | `gview_cameraImage` | 图像显示 |
| `QProgressBar` | `progress_` | `progress_task` | 任务进度 |
| `QSlider` | `slider_` | `slider_displayGain` | 连续调节 |
| `QGroupBox` | `group_` | `group_cameraStatus` | 分组区域 |
| `QTabWidget` | `tab_` | `tab_mainConfig` | 并列功能页 |
| `QStackedWidget` | `stack_` | `stack_workflow` | 流程页面切换 |
| `QSplitter` | `splitter_` | `splitter_mainView` | 可拖拽分割区域 |
| `QFrame` | `frame_` | `frame_statusLight` | 装饰、状态灯、分隔线 |
| `QScrollArea` | `scroll_` | `scroll_paramPanel` | 参数过多时滚动显示 |
| `QLayout` | `layout_` | `layout_cameraControl` | 仅当代码中访问布局时命名 |
| `图表显示` | `plot_` | `plot_cameraVolt` | 显示折线图、 柱状图、饼图、圆环图、雷达图、热力图等所有二维可视化图像控件的命名|

### 12.4 QPushButton 按钮规范

#### 12.4.1 普通按钮

- 普通按钮使用 `clicked()`。
- 槽函数只负责触发动作，不直接执行耗时逻辑。
- 按钮点击后如涉及 RPC、文件 IO、图像处理等耗时动作，必须立即禁用按钮防止重复点击。

```cpp
void CameraControlPanel::on_btn_captureImage_clicked()
{
    setCameraUiBusy(true);
    emit requestCaptureImage();
}
```

#### 12.4.2 切换按钮

- 切换类按钮必须设置 `checkable=true`。
- 切换类按钮使用 `clicked(bool checked)`。
- `checked` 只能表示按钮当前勾选状态，真正的设备状态必须以后端返回结果为准。
- 按钮文字必须随状态变化，但业务判断不得依赖按钮文字。

```cpp
void CameraControlPanel::on_btn_connectGrabber_clicked(bool checked)
{
    if (checked) 
	{
        ui->btn_connectGrabber->setText(tr("Disconnect Grabber"));
        emit requestConnectGrabber();
    }
	else 
	{
        ui->btn_connectGrabber->setText(tr("Connect Grabber"));
        emit requestDisconnectGrabber();
    }
}
```

#### 12.4.3 后端失败时必须回滚 UI 状态

### 12.5 QLabel 显示规范

#### 12.5.1 静态 Label

- 仅用于标题、单位、固定说明。
- 不需要在代码中访问的静态 Label 可以不强制命名。
- 静态 Label 的文本应简短，避免在 Label 中写大段说明。

```text
Camera Status:
Exposure Time:
Temperature:
```

#### 12.5.2 动态 Label

- 动态 Label 必须命名。
- 初始值必须明确，禁止空白。
- 数值显示必须带单位。
- 无效值统一显示 `--` 或 `N/A`。
- 状态类 Label 建议集中通过状态更新函数刷新。

```cpp
void CameraControlPanel::updateCameraStatus(const CameraStatus& status)
{
    ui->label_cameraState->setText(status.isOpen ? tr("Open") : tr("Closed"));

    if (status.hasTemperature) 
	{
        ui->label_cameraTemperature->setText(
            QString("%1 °C").arg(status.temperatureDegreeC, 0, 'f', 1));
    } else 
	{
        ui->label_cameraTemperature->setText(tr("-- °C"));
    }
}
```

---

### 12.6 输入类控件规范

#### 12.6.1 QLineEdit

- 仅用于文本、编号、路径等输入。
- 数值输入优先使用 `QSpinBox` 或 `QDoubleSpinBox`，不推荐用 `QLineEdit` 手动解析数字。
- 必须设置 `placeholderText` 或默认值。
- 有格式要求时必须设置 `QValidator`。
- 读取输入前必须 `trimmed()`，去除字符串首尾的空白字符。

```cpp
void CameraControlPanel::initLotIdEdit()
{
    QRegularExpression regExp("[A-Za-z0-9_\\-]{1,32}");
    ui->edit_lotId->setValidator(new QRegularExpressionValidator(regExp, ui->edit_lotId));
    ui->edit_lotId->setPlaceholderText(tr("Input lot id"));
    ui->edit_lotId->setClearButtonEnabled(true);
}

QString CameraControlPanel::lotId() const
{
    return ui->edit_lotId->text().trimmed();
}
```

#### 12.6.2 QSpinBox / QDoubleSpinBox

- 数值输入必须设置范围、步进、精度和单位后缀。
- 物理量变量和控件名必须体现单位。

```cpp
void CameraControlPanel::initExposureSpinBox()
{
    ui->doubleSpin_exposureTimeMs->setRange(0.01, 1000.0);
    ui->doubleSpin_exposureTimeMs->setDecimals(2);
    ui->doubleSpin_exposureTimeMs->setSingleStep(0.1);
    ui->doubleSpin_exposureTimeMs->setSuffix(tr(" ms"));
}
```

#### 12.6.3 QComboBox

- 用于枚举选择。
- `itemData` 必须绑定枚举值，禁止通过显示文本判断业务状态。
- 切换后应通过规范槽函数发出业务请求。

```cpp
enum class CameraSource {
    LOCAL = 0,
    EXTERNAL = 1
};

void CameraControlPanel::initCameraSourceCombo()
{
    ui->combo_cameraSource->addItem(tr("LOCAL"), static_cast<int>(CameraSource::LOCAL));
    ui->combo_cameraSource->addItem(tr("EXTERNAL"), static_cast<int>(CameraSource::EXTERNAL));
}

void CameraControlPanel::on_combo_cameraSource_currentIndexChanged(int index)
{
    const int value = ui->combo_cameraSource->itemData(index).toInt();
    emit requestSetCameraSource(static_cast<CameraSource>(value));
}
```

---

### 12.7 选择类控件规范

#### 12.7.1 QCheckBox

- 用于独立开关项。
- 命名必须使用 `check_`。
- 业务状态不得只保存在 checkbox 中，必须同步到配置或状态对象。

```cpp
void CameraControlPanel::on_check_enableAutoSave_toggled(bool checked)
{
    emit requestSetAutoSaveEnabled(checked);
}
```

#### 12.7.2 QRadioButton

- 用于互斥选项。
- 多个 RadioButton 必须通过 `QButtonGroup` 管理。

```cpp
void CameraControlPanel::initSourceRadioButtons()
{
    m_sourceButtonGroup = new QButtonGroup(this);
    m_sourceButtonGroup->addButton(ui->radio_localSource, static_cast<int>(CameraSource::LOCAL));
    m_sourceButtonGroup->addButton(ui->radio_externalSource, static_cast<int>(CameraSource::EXTERNAL));

    connect(m_sourceButtonGroup,
            &QButtonGroup::idClicked,
            this,
            &CameraControlPanel::onSourceRadioButtonClicked);
}

void CameraControlPanel::onSourceRadioButtonClicked(int id)
{
    emit requestSetCameraSource(static_cast<CameraSource>(id));
}
```

---

### 12.8 表格与列表控件规范

#### 12.8.1 QTableWidget

- 仅适用于小数据量、简单表格。
- 建议数据量：不超过 1000 行。
- 表格默认应禁止直接编辑，除非明确是参数编辑表。
- 表格列宽、选择模式、排序行为必须初始化。
- 禁止在大量单元格中嵌入 `QWidget`，否则性能和内存开销较大。

```cpp
void ResultPanel::initResultTable()
{
    ui->table_resultList->setColumnCount(4);
    ui->table_resultList->setHorizontalHeaderLabels({
        tr("Index"),
        tr("Code"),
        tr("Message"),
        tr("Time")
    });

    ui->table_resultList->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->table_resultList->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->table_resultList->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->table_resultList->horizontalHeader()->setStretchLastSection(true);
}
```

#### 12.8.2 QTableView / Model-View

- 大数据量、实时刷新数据、筛选排序数据必须使用 `MVC（模型-视图-控制器）架构`。
- Model 中保存数据，View 只负责显示，Controller负责接收View的数据，选择合适的 View 展示结果。
- 禁止在 UI 层直接维护复杂二维业务数据，复杂业务数据处理建议新开线程放置在后台处理。

```cpp
class TaskTableModel : public QAbstractTableModel {
    Q_OBJECT

public:
    explicit TaskTableModel(QObject* parent = nullptr);

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;

    void setTasks(const QVector<TaskInfo>& tasks);

private:
    QVector<TaskInfo> m_tasks;
};
```

---

### 12.9 图像显示规范

#### 12.9.1 控件选择

- 大图显示、相机图像、ROI 绘制、缩放、拖拽、叠加十字线等场景必须使用 `QGraphicsView`。
- `QLabel` 仅适合显示小图标、状态图标、Logo 或简单缩略图。
- 禁止在 UI 线程中执行大规模图像算法。

#### 12.9.2 QGraphicsView 初始化示例

```cpp
void ImageViewerWidget::initGraphicsView()
{
    m_scene = new QGraphicsScene(this);
    m_imageItem = new QGraphicsPixmapItem();

    m_scene->addItem(m_imageItem);
    ui->graphicsView_cameraImage->setScene(m_scene);
    ui->graphicsView_cameraImage->setRenderHint(QPainter::Antialiasing, false);
    ui->graphicsView_cameraImage->setDragMode(QGraphicsView::ScrollHandDrag);
    ui->graphicsView_cameraImage->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    ui->graphicsView_cameraImage->setResizeAnchor(QGraphicsView::AnchorViewCenter);
}
```

#### 12.9.3 图像刷新示例

```cpp
void ImageViewerWidget::updateImage(const QImage& image)
{
    if (image.isNull()) {
        return;
    }

    m_imageItem->setPixmap(QPixmap::fromImage(image));
    m_scene->setSceneRect(image.rect());

    if (m_isFirstImage) {
        ui->graphicsView_cameraImage->fitInView(m_imageItem, Qt::KeepAspectRatio);
        m_isFirstImage = false;
    }
}
```

---

### 12.10 布局规范

#### 12.10.1 总体要求

- 顶层窗口必须使用布局管理器。
- 非必要不要使用固定坐标摆放控件。
- 仅状态灯、固定图标、固定尺寸按钮允许使用固定尺寸。
- 等距参数面板建议使用 `QGridLayout` 或 `QFormLayout`。
- 控件过多时应使用 `QScrollArea`，避免窗口被迫无限放大。

#### 12.10.2 Layout 使用建议

| 布局 | 推荐场景 | 限制 |
|---|---|---|
| `QVBoxLayout` | 垂直堆叠区域 | 不要嵌套过深 |
| `QHBoxLayout` | 横向按钮、状态栏 | 按钮过多时应分组 |
| `QGridLayout` | 参数名 + 输入框 + 单位 | 行列必须对齐 |
| `QScrollArea` | 参数很多的面板 | 避免嵌套滚动区域 |
| `QStackedWidget` | 工作流页面切换 | 页面切换必须有明确状态 |
| `QTabWidget` | 同级配置页 | 不用于流程步骤 |

---

### 12.11 状态显示与控件使能规范

#### 12.11.1 状态更新集中化

- 禁止在多个槽函数中分散更新同一组控件状态。
- 必须使用统一的 `updateXxxUiState()` 函数更新按钮、Label、输入框可用性。
- UI 状态必须以后端状态或本地状态机为准。

```cpp
void CameraControlPanel::updateCameraUiState(const CameraStatus& status)
{
    ui->label_cameraState->setText(status.isOpen ? tr("Open") : tr("Closed"));

    ui->btn_openCamera->setEnabled(!status.isOpen && !status.isBusy);
    ui->btn_closeCamera->setEnabled(status.isOpen && !status.isBusy);
    ui->btn_captureImage->setEnabled(status.isOpen && !status.isBusy);

    ui->combo_cameraSource->setEnabled(!status.isOpen && !status.isBusy);
    ui->doubleSpin_exposureTimeMs->setEnabled(status.isOpen && !status.isBusy);
}
```

#### 12.11.2 Busy 状态

- 执行 RPC、采集、保存、加载等操作时必须进入 Busy 状态。
- Busy 状态下应禁用相关按钮，必要时显示进度条或等待提示。
- 操作完成或失败都必须恢复控件状态。

```cpp
void CameraControlPanel::setCameraUiBusy(bool isBusy)
{
    ui->btn_openCamera->setEnabled(!isBusy);
    ui->btn_closeCamera->setEnabled(!isBusy);
    ui->btn_captureImage->setEnabled(!isBusy);
    ui->progress_cameraOperation->setVisible(isBusy);
}
```

---

### 12.12 信号与槽规范

#### 12.12.1 基本规则

- [禁止] 使用 lambda 表达式连接信号与槽。
- [禁止] 在槽函数中写大量业务逻辑。
- [禁止] 一个槽函数处理多个无关控件的业务。
- [禁止] 禁止使用SIGNAL/SLOT旧版写法，旧版没有类型检查。
- 槽函数必须有明确声明，优先放在 `private slots:` 中。
- 槽函数与普通函数一样必须有要有注释。

```cpp
// [推荐]
connect(ui->btn_openCamera,
        &QPushButton::clicked,
        this,
        &CameraControlPanel::onOpenCameraButtonClicked);

// [禁止]
connect(ui->btn_openCamera, SIGNAL(clicked()), this, SLOT(onOpenCameraButtonClicked()));

// [禁止]
connect(ui->btn_openCamera, &QPushButton::clicked, this, [this]() {
    openCamera();
});
```

#### 12.12.2 Designer 自动槽函数

使用 Qt Designer 自动连接时，记得手动添加说明：

```cpp
private slots:
	//打开相机
    void on_btn_openCamera_clicked();
	//连接采集卡
    void on_btn_connectGrabber_clicked(bool checked);
```

#### 12.12.3 自定义连接槽函数时

手动连接时，信号与槽函数命名要避免和 Designer 自动生成的混用。

```cpp
signals:
	void callOpenCameraClicked();

private slots:
    
    void onCaptureImageClicked();
		
```

#### 12.12.4 跨线程信号槽

- Worker 线程不得直接访问 UI 控件。
- Worker 线程只通过 signal 把结果发送给 UI。
- 跨线程连接必须使用 `Qt::QueuedConnection` 或依赖 AutoConnection 的队列行为。

```cpp
connect(m_rpcWorker,
        &RpcClientWorker::cameraStatusUpdated,
        this,
        &CameraControlPanel::onCameraStatusUpdated,
        Qt::QueuedConnection);
```

---

### 12.13 UI 线程与后台任务规范

#### 12.13.1 禁止 UI 线程执行耗时逻辑

以下操作禁止在 UI 线程中直接执行：

- gRPC 同步阻塞调用。
- 相机采集阻塞等待。
- 大图像算法处理。
- 文件批量读写。
- 数据库大量查询。
- 循环等待设备状态。
- `std::this_thread::sleep_for()` 或 `QThread::sleep()`。
- 非阻塞UI延时代码示例（注意，该代码不会阻止新的信号发送，禁止递归）
```
void delaytime(int tms)
{
    QEventLoop loop;
    QTimer::singleShot(tms, &loop, &QEventLoop::quit);
    loop.exec();
}
```


#### 12.13.2 推荐结构

```text
UI Thread
  ├─ 显示界面
  ├─ 收集用户输入
  ├─ 发出 requestXxx 信号
  └─ 接收 worker 返回结果并刷新界面

RPC / Worker Thread
  ├─ 连接设备服务
  ├─ 执行 gRPC 调用
  ├─ 接收状态订阅
  └─ 通过 signal 返回结果
```

### 12.14 弹窗、错误提示与日志规范

- 普通错误使用状态栏、错误 Label 或日志区显示。
- 阻塞式 `QMessageBox` 仅用于用户必须确认的严重错误或危险操作，非必要不要使用。
- 错误显示必须包含错误码、模块名和简短说明。
- 不允许只提示“失败”，必须说明失败对象和错误码。
- 重复错误应节流，避免连续弹窗干扰操作。

```cpp
void MainWindow::showErrorMessage(int errorCode, const QString& message)
{
    const QString text = tr("Error Code: %1\nMessage: %2").arg(errorCode).arg(message);

    ui->statusbar->showMessage(text, 5000);
    appendLogMessage(LogLevel::Error, text);
}
```

---

### 12.15 样式、图标与资源规范

- 图标、图片、QSS 文件必须放入 `.qrc` 资源文件。
- 禁止在代码中使用本机绝对路径加载资源。
- 状态颜色必须有统一定义，例如正常、警告、错误、离线。
- 样式不能作为业务状态判断依据。

```cpp
// [推荐]
ui->btn_openCamera->setIcon(QIcon(":/icons/camera_open.svg"));

// [禁止]
ui->btn_openCamera->setIcon(QIcon("D:/project/icons/camera_open.svg"));
```

```cpp
void MainWindow::loadStyleSheet()
{
    QFile file(":/qss/main_window.qss");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return;
    }

    qApp->setStyleSheet(QString::fromUtf8(file.readAll()));
}
```

---

### 12.16 国际化与显示文本规范

- UI 显示文本必须使用 `tr()` 包裹。
- 不推荐在代码中拼接大量中文或英文长句。
- 数值显示必须明确格式和单位。
- 时间显示必须使用统一格式。

```cpp
ui->label_cameraTemperature->setText(
    tr("%1 °C").arg(status.temperatureDegreeC, 0, 'f', 1));

ui->label_updateTime->setText(
    QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"));
```

---

### 12.17 Designer 使用规范

- `.ui` 文件只负责布局和基础属性，不写业务逻辑。
- 使用 Designer 后必须检查：
  - 是否存在重复 `objectName`。
  - 是否存在默认无意义命名且被代码访问的控件。
  - Tab 顺序是否合理。
  - Layout 是否完整。
  - 控件是否设置了合适的 `sizePolicy`。
  - 切换按钮是否设置了 `checkable=true`。
- 复杂页面建议拆分为多个 `QWidget` 子页面，禁止单个 `.ui` 文件无限膨胀。
- 不建议手动修改 `.ui` XML，除非必须解决冲突或修复异常。

---

### 12.18 控件限制与建议汇总

| 控件 / 布局 | 使用限制 | 推荐做法 |
|---|---|---|
| `QLabel` | 不适合大图显示和复杂交互 | 状态值、单位、标题、图标 |
| `QPushButton` | 不直接执行耗时逻辑 | 发出请求信号，等待 worker 返回 |
| `QLineEdit` | 不推荐做数值输入 | 文本输入 + Validator |
| `QSpinBox` / `QDoubleSpinBox` | 必须设置范围和单位 | 替代手写数字解析 |
| `QComboBox` | 不通过显示文本判断业务状态 | 使用 `itemData` 绑定枚举 |
| `QTableWidget` | 不适合大数据和高频刷新 | 小表格、静态结果 |
| `QTableView` | 需要额外 Model | 大数据、结果列表、任务列表 |
| `QGraphicsView` | 初始化略复杂 | 大图、相机图像、ROI、缩放 |
| `QProgressBar` | 不代表设备真实状态 | 只表示任务进度或 Busy |
| `QTimer` | timeout 中禁止耗时操作 | 定时触发请求或轻量刷新 |
| `QTabWidget` | 不适合流程控制 | 同级配置页 |
| `QStackedWidget` | 页面切换必须受状态控制 | 向导流程、模式切换 |
| `QSplitter` | 不用于普通按钮布局 | 主界面区域分割 |
| `QScrollArea` | 避免嵌套滚动 | 大量参数面板 |

---

### 12.21 Qt 前端禁止事项

- [禁止] UI 线程中直接执行耗时 RPC、硬件访问、图像算法和大文件 IO。
- [禁止] Worker 线程直接访问 `ui->xxx` 控件。
- [禁止] 使用 lambda 表达式绑定信号与槽。
- [禁止] 使用 `SIGNAL()` / `SLOT()` 字符串式旧语法。
- [禁止] 通过按钮文字、Label 文字、QSS 样式判断业务状态。
- [禁止] 动态控件使用默认 `objectName`。
- [禁止] 重复 `objectName`。
- [禁止] 大图显示使用 QLabel 承担缩放、拖拽、ROI 等复杂功能。
- [禁止] 没有范围限制的数值输入。
- [禁止] 使用本地绝对路径加载图标、图片、QSS。
- [禁止] 用大量 `setFixedSize()` 固定界面。
- [禁止] 在槽函数中堆积大量业务逻辑。
- [禁止] 弹窗提示只写“失败”而没有错误码和模块信息。
