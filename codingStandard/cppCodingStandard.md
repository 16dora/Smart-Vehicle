# C++ 20 编码规范（Smart Vehicle 项目）

**版本**: 1.1  
**适用范围**: 
- Smart Vehicle ESP32 固件及配套工具
- 设备通信与状态管理软件
- 设备控制 / 运动控制 / 算法模块

**语言标准**: C++20

---

## 核心理念

1. **可读性 > 技巧性**：代码是写给人看的，其次才是给机器执行的。
2. **确定性 > 性能**：在工业控制中，行为的确定性比极致的性能更重要。
3. **可追溯 > 简洁**：日志、状态和错误信息必须完整可追溯。

---

## 一、文件与命名空间

### 1.1 文件命名
- **格式**：全小写，下划线分隔。
- **扩展名**：头文件 `.h`，源文件 `.cpp`。

```cpp
// [推荐]
device_controller.h
stage_controller.cpp

// [禁止]
DeviceController.h
StageController.CPP
```

### 1.2 命名空间
- **层级化**：使用嵌套命名空间反映模块结构。
- **头文件限制**：禁止在头文件中使用 `using namespace std;`。
- **匿名空间**：禁止在匿名 namespace 中定义跨编译单元共享的逻辑。

```cpp
namespace vehicle {
namespace stage {

class Controller;

} // namespace stage
} // namespace vehicle
```

---

## 二、命名规范（强制）

### 2.1 成员变量
- **前缀**：统一使用 `m_` 前缀。
- **风格**：小驼峰命名法（camelCase）。

| 类型 | 示例 | 说明 |
|------|------|------|
| 普通变量 | `m_imageWidth` | 语义清晰 |
| 布尔变量 | `m_isOpen`, `m_hasError` | 必须以 `is/has/can` 开头 |
| 计数 | `m_retryCount` | 明确单位或含义 |
| 时间 | `m_timeoutMs` | 必须带单位后缀 |
| 指针 | `m_bufferPtr` | 显式标识指针类型 |

```cpp
class Camera {
private:
    int m_width;              // [符合]
    double m_exposureTimeMs;  // [符合] 带单位
    bool m_isEnabled;         // [符合] 疑问句式
    
    int m_a;                  // [禁止] 无意义命名
};
```

### 2.2 函数命名
- **动词开头**：表示动作或行为。
- **布尔函数**：必须是疑问句形式（`is`, `has`, `can`）。
```cpp
void startOperation();      // [符合] 动词开头
bool isDeviceReady() const;   // [符合] 疑问句式
```

### 2.3 注释
- **注释**:申明和实现都必须要有注释，注释要简单明确，格式如下：
```
    // 任务调度主循环。
    // 输入：无。
    // 输出：无。
    // 作用：只负责把 设备任务队列中的任务转交给线程池，不直接执行下游 RPC。
    void runTaskDispatchLoop();
```
- **注释比例**:注释不要过于繁琐，比例控制在10%~20%左右，成员变量定义的后面或者上面最好有注释。
```
    // 设备服务地址
    QString m_deviceAddress;
```
- **全局声明强制注释**：所有文件作用域或命名空间作用域的变量、`static constexpr` 常量、枚举类型、枚举项和错误码定义，都必须逐项添加明确的中文注释。注释必须说明用途，并按实际情况说明单位、取值语义或错误触发条件；禁止只写“常量”“状态”或“错误码”等无具体含义的描述。该要求不受注释比例建议限制。

```cpp
// 错误码 2101：已有设备操作正在执行，当前请求无法受理。
static constexpr int RESULT_BUSY = 2101;

// 设备连接状态。
enum class DeviceConnectionState
{
    // 尚未建立设备连接。
    DISCONNECTED,
    // 已建立设备连接。
    CONNECTED
};
```


### 2.3 类与结构体
- **类名**：大驼峰命名法（PascalCase）。
- **结构体**：仅用于纯数据载体（POD），不使用成员函数逻辑。

```cpp
class StageController;        // [符合] 类
struct ImageSize {            // [符合] 纯数据结构
    int width;
    int height;
};
```

### 2.4 常量
- **定义方式**：使用 `static constexpr`，名称必须要大写，并逐项添加说明用途、单位或取值语义的中文注释。
- **禁止**：使用 `#define` 定义常量。

```cpp
// 失败后允许重试的最大次数。
static constexpr int MAX_RETRY_COUNT = 5;
// 未配置增益时使用的默认增益倍数。
static constexpr double DEFAULT_GAIN = 1.0;
```

---

## 三、类与对象生命周期

### 3.1 成员变量访问控制
- **原则**：所有成员变量必须为 `private`。
- **例外**：仅当 `struct` 为纯数据载体（POD）时允许 `public`。

```cpp
class Device 
{
public:
    int value() const;
    void setValue(int v);

private:
    int m_value; // [符合] 私有成员
};
```

### 3.3 Rule of Zero / Five
- **默认析构**：若无需特殊资源管理，使用 `= default`。
- **禁止拷贝**：对于独占资源的类，显式删除拷贝构造和赋值运算符。

```cpp
~Camera() = default;
Camera(const Camera&) = delete;
Camera& operator=(const Camera&) = delete;
```

---

## 四、函数规范

### 4.1 const 正确性（强制）
- **Getter**：必须标记为 `const`。
- **不修改状态**：任何不改变对象状态的成员函数必须标记为 `const`。

```cpp
int width() const;      // [符合]
bool isValid() const;   // [符合]
```


---

## 八、多线程与并发

### 8.1 成员变量线程安全
- **互斥锁**：使用 `mutable std::mutex` 保护共享数据。
- **归属明确**：注释说明每个变量的读写线程归属。

```cpp
mutable std::mutex m_mutex;

double m_targetPos;   // UI thread write
double m_currentPos;  // RT thread read
```

### 8.2 禁止事项
- [禁止] 隐式共享数据。
- [禁止] 不加锁访问共享成员。
- [禁止] 使用 `volatile` 进行线程同步（应使用 `std::atomic`）。

---

## 九、代码风格

### 9.1 缩进与空格
- **缩进**：4 空格，禁止使用 Tab。
- **大括号**：微软风格（左大括号必须换行）。
- **建议使用微软风格，更加现代化，同时可读性更高，面对多层嵌套时，更好方便确定每个花括号的作用域范围**
```cpp
if (m_isEnabled) 
{
    doWork();
}
```


---

## 十、禁止事项（红线）

- [禁止] **全局变量**：破坏封装性，难以追踪状态。
- [禁止] **goto**：导致控制流混乱。
- [禁止] **宏定义常量**：使用 `constexpr` 替代。
- [禁止] **头文件中的 using namespace std;**：污染命名空间。
- [禁止] **public 成员变量**：破坏封装（POD struct 除外）。
- [禁止] **裸指针长期持有资源**：易导致内存泄漏。
- [禁止] **依赖未定义行为**：确保代码在所有编译器下行为一致。

---

## 十一、Smart Vehicle 项目补充

### 11.1 数值与物理量规范
- **单位后缀**：所有物理量变量名必须包含单位。
- **强类型封装**：推荐使用 `struct` 封装物理量，防止单位混淆。
- **浮点比较**：必须使用误差容忍（Epsilon）。

```cpp
double m_exposureTimeMs;        // [符合] 带单位
double m_temperatureDegreeC;    // [符合] 带单位

struct ExposureTimeMs {         // [符合] 强类型
    double value;
};

constexpr double EPS = 1e-6;
if (std::abs(a - b) < EPS) { ... }
```

### 11.5 日志与可追溯性
- **日志内容**：必须包含时间戳、参数版本、错误码。
- **关键节点**：操作开始、结束、参数变更必须记录日志。
- **统一入口（2026-10-15）**：固件业务日志统一使用 `esp_log.h` 的 `ESP_LOGI/W/E/D` 宏，不另建 logger 或文件日志。
- **初始化顺序**：`app_main()` 中先配置所需日志输出通道，再读取业务配置、构造模块和启动业务任务；需要显式初始化的通道只有初始化成功才继续业务。
- **初始化失败**：自定义日志输出通道初始化失败时，仅在该路径直接输出具体原因，并停止后续业务启动；不依赖回退 logger 继续运行。
- **日志打印限制**：除上述初始化失败退出路径外，不再新增 `std::cout/cerr/clog`、`printf/fprintf`、`puts/perror` 等日志打印。字符串流、正常文件读写及第三方库内部实现不因本规则改动；未涉及的历史代码不批量迁移。
- **级别与字段**：正常流程和成功结果用 INFO，可恢复问题、重试和部分成功用 WARN，终止性失败和异常用 ERROR；调试细节用 DEBUG，严重故障使用 ERROR 并说明原因。保留关键函数首尾、第三方调用前后及必要参数、结果和错误详情，不擅自删除字段或降低日志频率。
- **名称与格式**：固定日志标签提供模块标识，正文仅添加必要访问模式；日志格式、级别和输出通道集中管理。

```cpp
// 正常业务入口和成功退出使用 INFO，失败退出按语义使用 WARN 或 ERROR。
ESP_LOGI(TAG, "runOperation() >> version=%s param_set=%s", version, paramSet);
ESP_LOGI(TAG, "runOperation() << result=%d", result);
ESP_LOGE(TAG, "runOperation() << result=%d message=%s", result, errorMessage);

// 模块名称由日志标签提供，模式标识只在正文中输出一次。
ESP_LOGI(TAG, "[SDK] connect() >> ip=%s port=%d", ip, port);
```
