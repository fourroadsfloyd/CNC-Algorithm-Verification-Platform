# CNC-Algorithm-Verification-Platform
## 1. 系统概述

### 1.1 平台架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        主应用程序层                              │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────────────┐  ┌─────────────┐             │
│  │  输入模块    │  │   算法框架层         │  │  输出模块    │             │
│  │  (NURBS/    │  │  ┌─────────────┐   │  │  (G代码     │             │
│  │   线段)     │──│──│ 全局拟合    │   │──│──  生成)    │             │
│  └─────────────┘  │  │ 局部光顺    │   │  └─────────────┘             │
│                   │  │ 速度控制    │   │                               │
│                   │  └─────────────┘   │                               │
│                   └─────────────────────┘                               │
├─────────────────────────────────────────────────────────────────┤
│                     可视化层 (QtOpenGL)                          │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │   轨迹可视化    │  │   刀具可视化     │  │   加工仿真      │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                     核心数据结构层                               │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐           │
│  │CurvePath │ │ToolPose  │ │GCodeCmd  │ │MachineCtx│           │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘           │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 目录结构

```
cnc_simulator/
├── include/
│   ├── core/                    # 核心数据结构
│   │   ├── curve_path.hpp       # 曲线/路径数据结构
│   │   ├── tool_pose.hpp       # 刀具位姿
│   │   ├── gcode_command.hpp   # G代码命令
│   │   ├── machine_context.hpp # 机床上下文
│   │   └── interpolation.hpp   # 插补相关
│   │
│   ├── algorithms/              # 算法框架
│   │   ├── fitting/            # 全局拟合
│   │   │   ├── fitting_algorithm.hpp
│   │   │   ├── global_fitting_factory.hpp
│   │   │   └── nurbs_fitting.hpp
│   │   │
│   │   ├── smoothing/          # 局部光顺
│   │   │   ├── smoothing_algorithm.hpp
│   │   │   ├── smoothing_factory.hpp
│   │   │   └── parametric_smoothing.hpp
│   │   │
│   │   └── speed_control/      # 速度控制
│   │       ├── speed_algorithm.hpp
│   │       ├── speed_factory.hpp
│   │       └── chordal_speed.hpp
│   │
│   ├── machines/                # 机床模型
│   │   ├── machine_base.hpp     # 机床基类
│   │   ├── three_axis_machine.hpp
│   │   └── five_axis_machine.hpp # 预留五轴扩展
│   │
│   └── visualization/          # 可视化
│       ├── gl_widget.hpp
│       ├── tool_renderer.hpp
│       └── path_renderer.hpp
│
├── src/
├── plugins/                     # 插件目录(用于扩展算法)
└── CMakeLists.txt
```

---

## 2. 核心数据结构

### 2.1 曲线与路径

```cpp
// 基础点结构
struct Point3D {
    double x, y, z;
    double feedrate = 0.0;        // 进给速度
    double curvature = 0.0;      // 曲率(计算得出)
};

struct Point5D {
    Point3D linear;
    double a = 0.0, b = 0.0, c = 0.0;  // 旋转轴角度(五轴预留)
};

// NURBS曲线定义
struct NURBSCurve {
    std::vector<double> knots;           // 节点矢量
    std::vector<Point3D> control_points; // 控制点
    std::vector<double> weights;         // 权因子
    int degree;                          // 基函数阶数

    // 基础接口
    Point3D evaluate(double t) const;    // 计算t处点
    Point3D derivative(double t, int order) const;  // 求导
};

// 离散路径(所有算法的通用中间格式)
struct PathSegment {
    Point3D start;
    Point3D end;
    double chord_length;
    double arclength;
    double max_curvature;
    int segmentation_level = 0;
};

struct DiscretizedPath {
    std::vector<PathSegment> segments;
    double total_length;
    std::vector<double> accumulated_length;  // 弧长参数化用

    // 弧长参数化查询
    double queryParameterByLength(double s) const;
};
```

### 2.2 刀具位姿

```cpp
// 刀具位姿基类
class ToolPose {
public:
    virtual ~ToolPose() = default;

    Point3D position;          // 位置
    Vec3D direction;          // 刀轴方向
    Vec3D normal;              // 加工法向(切屑厚度计算用)

    // 运动学正逆解
    virtual std::vector<double> forwardKinematics() const = 0;
    virtual void inverseKinematics(const std::vector<double>& axis_values) = 0;
};

// 三轴刀具位姿
class ToolPose3Axis : public ToolPose {
public:
    ToolPose3Axis();

    // 机床坐标(电机反馈值)
    struct MachinePosition {
        double x, y, z;       // 直线轴
    };

    MachinePosition machine_pos;

    std::vector<double> forwardKinematics() const override;
    void inverseKinematics(const std::vector<double>& axis_values) override;
};

// 五轴刀具位姿(预留)
class ToolPose5Axis : public ToolPose {
public:
    // 机床坐标
    struct MachinePosition {
        double x, y, z;       // 直线轴
        double a, b;          // 旋转轴(可配置为AB/BC/AC)
    };

    MachinePosition machine_pos;

    // 工件坐标(刀尖点在工件坐标系的位置)
    Point3D workpiece_pos;
    // 刀轴方向(在工件坐标系中的方向向量)
    Vec3D tool_direction;

    std::vector<double> forwardKinematics() const override;
    void inverseKinematics(const std::vector<double>& axis_values) override;

    // 五轴刀具补偿
    double tool_length_offset;
    double tool_diameter;
};
```

### 2.3 G代码命令

```cpp
// G代码命令类型
enum class GCodeType {
    G00,  // 快速定位
    G01,  // 直线插补
    G02,  // 顺时针圆弧
    G03,  // 逆时针圆弧
    G05,  // 光顺插补(PVT模式)
    G64,  // 连续路径模式(保持速度)
    // ... 其他
};

// 通用G代码参数
struct GCodeModal {
    GCodeType type = GCodeType::G01;
    double feedrate = 0.0;
    double previous_feedrate = 0.0;

    // 坐标系统
    int g54_select = 0;  // 工件坐标系选择
    bool absolute_mode = true;

    // 速度模式
    enum class FeedrateMode { UNITS_PER_MIN, UNITS_PER_REV };
    FeedrateMode feed_mode = FeedrateMode::UNITS_PER_MIN;
};

// 单条G代码行
struct GCodeCommand {
    GCodeType type;

    // 目标位置(取决于G90/G91)
    std::optional<Point3D> target;
    std::optional<Point5D> target_5axis;  // 五轴扩展

    // 插补参数
    double feedrate;
    std::vector<double> pqr;  // 圆弧参数等

    // 原始文本(保留格式)
    std::string original_line;
    int line_number;

    // 元信息
    double estimated_time = 0.0;  // 预估执行时间
    double smooth_curvature = 0.0; // 光顺后的最大曲率
};

// G代码程序
struct GCodeProgram {
    std::vector<GCodeCommand> commands;
    std::unordered_map<int, std::string> line_labels;  // 行号->标签

    // 程序头尾
    std::string header;  // G54, G64, T1 M6等
    std::string footer;  // M30等

    // 统计信息
    double total_time;
    double total_length;
};
```

### 2.4 机床上下文

```cpp
// 机床轴配置
struct AxisConfig {
    double max_velocity;      // 最大速度 mm/min
    double max_acceleration;  // 最大加速度 mm/s²
    double max_jerk;          // 最大加加速度 mm/s³
    double backlash;          // 反向间隙
    double resolution;        // 位置分辨率
};

// 机床运动学模型
struct KinematicsModel {
    enum class Type {
        ThreeAxisCartesian,
        ThreeAxisCartesianWithTrunnion,
        FiveAxisTableTable,
        FiveAxisTableTool,
        FiveAxisDualTable
    };
    Type type;
};

// 机床上下文(贯穿整个加工过程)
class MachineContext {
public:
    // 轴配置
    std::vector<AxisConfig> axes;  // 通常6个轴(XYZABC)

    // 当前状态
    Point3D current_position;
    double current_feedrate;

    // 坐标系
    struct WorkCoordinate {
        Point3D origin_offset;
        Vec3D rotation_matrix;  // 旋转补偿
    };
    std::unordered_map<int, WorkCoordinate> work_coordinates;

    // 刀具信息
    struct Tool {
        double length;
        double diameter;
        int pocket;
    };
    Tool current_tool;

    // 运动学模型
    KinematicsModel kinematics;

    // 五轴后处理配置
    struct FiveAxisConfig {
        bool use_live_tooling = false;
        bool tilt_plane_compensation = true;
        double tool_tip_compensation_radius;
    };
    std::optional<FiveAxisConfig> five_axis_config;
};
```

---

## 3. 算法框架设计

### 3.1 全局拟合算法框架

全局拟合负责将离散的刀位点序列拟合为NURBS曲线，保证全局几何精度。

```cpp
// 全局拟合算法基类
class IGlobalFittingAlgorithm {
public:
    virtual ~IGlobalFittingAlgorithm() = default;

    // 算法标识
    virtual std::string name() const = 0;
    virtual std::string description() const = 0;

    // 算法参数
    struct Parameters {
        int max_control_points = 50;     // 最大控制点数量
        double tolerance = 0.001;        // 拟合公差 mm
        int degree = 3;                  // NURBS阶数
        bool keep_endpoints = true;     // 固定端点
    };
    virtual void setParameters(const Parameters& params) = 0;
    virtual Parameters getParameters() const = 0;

    // 核心拟合接口
    // 输入: 离散路径(通常是光顺后的)
    // 输出: NURBS曲线
    virtual NURBSCurve fit(const DiscretizedPath& path) = 0;

    // 误差分析(可选)
    virtual std::vector<double> computeDeviation(
        const DiscretizedPath& path,
        const NURBSCurve& curve
    ) = 0;
};

// 全局拟合工厂(用于运行时选择算法)
class GlobalFittingFactory {
public:
    using CreatorFunc = std::unique_ptr<IGlobalFittingAlgorithm>(*)();

    static GlobalFittingFactory& instance();

    // 注册算法
    void registerAlgorithm(const std::string& name, CreatorFunc creator);

    // 创建算法实例
    std::unique_ptr<IGlobalFittingAlgorithm> create(const std::string& name);

    // 获取所有可用算法
    std::vector<std::string> availableAlgorithms() const;

private:
    std::unordered_map<std::string, CreatorFunc> creators_;
};

// 注册宏(简化新算法注册)
#define REGISTER_FITTING_ALGORITHM(Name, ClassName) \
    static bool registered_##ClassName = []() { \
        GlobalFittingFactory::instance().registerAlgorithm( \
            Name, []() { return std::make_unique<ClassName>(); } \
        ); \
        return true; \
    }();
```

**内置拟合算法示例:**

```cpp
// 1. 全局G2样条拟合
class GlobalG2SplineFitting : public IGlobalFittingAlgorithm {
public:
    std::string name() const override { return "G2_Spline"; }
    std::string description() const override {
        return "全局G2连续样条拟合，保持曲率连续性";
    }

    void setParameters(const Parameters& params) override;
    Parameters getParameters() const override;

    NURBSCurve fit(const DiscretizedPath& path) override;
    std::vector<double> computeDeviation(
        const DiscretizedPath& path,
        const NURBSCurve& curve
    ) override;

private:
    Parameters params_;
    // 实现细节: 参数化 + 最小二乘求解
};

// 2. 基于NURBS的直接拟合
class DirectNURBSFitting : public IGlobalFittingAlgorithm {
    // 直接用控制点构造NURBS
};
```

### 3.2 局部光顺算法框架

局部光顺负责对离散路径进行平滑处理，消除振动和突变。

```cpp
// 局部光顺算法基类
class ILocalSmoothingAlgorithm {
public:
    virtual ~ILocalSmoothingAlgorithm() = default;

    virtual std::string name() const = 0;
    virtual std::string description() const = 0;

    struct Parameters {
        double smoothing_weight = 0.5;    // 光顺权重 [0,1]
        int window_size = 5;               // 窗口大小
        double max_curvature_change = 0.1; // 曲率变化限制
        double chord_error_tolerance = 0.001; // 弦高误差容忍
    };

    virtual void setParameters(const Parameters& params) = 0;
    virtual Parameters getParameters() const = 0;

    // 核心光顺接口
    // 输入: 原始离散路径
    // 输出: 光顺后的离散路径
    virtual DiscretizedPath smooth(const DiscretizedPath& input) = 0;

    // 迭代光顺(用于需要多次迭代的算法)
    virtual DiscretizedPath smoothIterative(
        const DiscretizedPath& input,
        int iterations
    ) = 0;

    // 局部编辑: 仅光顺指定区间
    virtual DiscretizedPath smoothRange(
        const DiscretizedPath& input,
        int start_idx,
        int end_idx
    ) = 0;
};

// 局部光顺工厂
class SmoothingFactory {
public:
    using CreatorFunc = std::unique_ptr<ILocalSmoothingAlgorithm>(*)();

    static SmoothingFactory& instance();
    void registerAlgorithm(const std::string& name, CreatorFunc creator);
    std::unique_ptr<ILocalSmoothingAlgorithm> create(const std::string& name);
    std::vector<std::string> availableAlgorithms() const;

private:
    std::unordered_map<std::string, CreatorFunc> creators_;
};

#define REGISTER_SMOOTHING_ALGORITHM(Name, ClassName) \
    static bool registered_##ClassName = []() { \
        SmoothingFactory::instance().registerAlgorithm( \
            Name, []() { return std::make_unique<ClassName>(); } \
        ); \
        return true; \
    }();
```

**内置光顺算法示例:**

```cpp
// 1. 参数样条光顺
class ParametricSplineSmoothing : public ILocalSmoothingAlgorithm {
public:
    std::string name() const override { return "Parametric_Spline"; }

    NURBSCurve fit(const DiscretizedPath& path) override;

private:
    Parameters params_;
};

// 2. 弧长样条光顺
class ArclengthSplineSmoothing : public ILocalSmoothingAlgorithm {
    // 基于弧长参数化的光顺
};

// 3. Savitzky-Golay滤波器
class SavitzkyGolaySmoothing : public ILocalSmoothingAlgorithm {
    // 保形滤波器
};

// 4. Chaikin's角落光顺
class ChaikinCornerSmoothing : public ILocalSmoothingAlgorithm {
    // 角落圆弧逼近
};

// 5. B样条磨光
class BSplineFairing : public ILocalSmoothingAlgorithm {
    // 最小二乘磨光
};
```

### 3.3 速度控制算法框架

速度控制根据路径特性(曲率、弦高误差等)规划进给速度。

```cpp
// 速度控制算法基类
class ISpeedControlAlgorithm {
public:
    virtual ~ISpeedControlAlgorithm() = default;

    virtual std::string name() const = 0;
    virtual std::string description() const = 0;

    struct Parameters {
        double max_feedrate = 5000.0;        // 最大进给 mm/min
        double min_feedrate = 50.0;          // 最小进给 mm/min
        double acceleration = 500.0;         // 最大加速度 mm/s²
        double jerk = 50.0;                  // 最大加加速度 mm/s³

        // 弦误差控制
        double chord_error = 0.005;           // 最大弦高误差 mm
        bool auto_chord_error = true;        // 自适应弦误差

        // 曲率限制
        double max_curvature_feedrate = 1000.0; // 曲率限速基准
        double curvature_power = 0.5;          // 曲率指数
    };

    virtual void setParameters(const Parameters& params) = 0;
    virtual Parameters getParameters() const = 0;

    // 核心速度规划接口
    // 输入: 离散路径 + 机床动力学限制
    // 输出: 带速度属性的离散路径
    virtual DiscretizedPath planSpeed(
        const DiscretizedPath& path,
        const MachineContext& machine
    ) = 0;

    // 速度限制计算
    virtual double computeSpeedLimit(
        double curvature,
        const Parameters& params
    ) const = 0;

    // 预查看速度(用于前瞻)
    virtual std::vector<double> previewSpeeds(
        const DiscretizedPath& path,
        int start_idx,
        int lookahead_count
    ) = 0;
};

// 速度控制工厂
class SpeedControlFactory {
public:
    using CreatorFunc = std::unique_ptr<ISpeedControlAlgorithm>(*)();

    static SpeedControlFactory& instance();
    void registerAlgorithm(const std::string& name, CreatorFunc creator);
    std::unique_ptr<ISpeedControlAlgorithm> create(const std::string& name);
    std::vector<std::string> availableAlgorithms() const;

private:
    std::unordered_map<std::string, CreatorFunc> creators_;
};

#define REGISTER_SPEED_ALGORITHM(Name, ClassName) \
    static bool registered_##ClassName = []() { \
        SpeedControlFactory::instance().registerAlgorithm( \
            Name, []() { return std::make_unique<ClassName>(); } \
        ); \
        return true; \
    }();
```

**内置速度控制算法示例:**

```cpp
// 1. 弦高误差限速(经典方法)
class ChordalSpeedControl : public ISpeedControlAlgorithm {
public:
    std::string name() const override { return "Chordal_Error"; }

    double computeSpeedLimit(double curvature, const Parameters& params) const override {
        // v = sqrt(2 * chord_error * curvature / (1 - chord_error * curvature))
        return std::sqrt(2 * params.chord_error * curvature /
               std::max(1e-6, 1 - params.chord_error * curvature));
    }

    DiscretizedPath planSpeed(const DiscretizedPath& path,
                              const MachineContext& machine) override;

private:
    Parameters params_;
};

// 2. 曲率限速
class CurvatureSpeedControl : public ISpeedControlAlgorithm {
    // v = k * (curvature ^ -power)
};

// 3. S型加减速(S曲线)
class SCurveSpeedControl : public ISpeedControlAlgorithm {
    // 包含加加速度限制的速度规划
};

// 4. 预测型速度规划(前瞻)
class PredictiveSpeedControl : public ISpeedControlAlgorithm {
    // 基于未来曲率的前瞻速度规划
};
```

---

## 4. 加工流程编排

### 4.1 流程管道

```
                输入(NURBS/线段)
                     	│
                     	▼
                ┌────────────────┐
                │  离散化模块      │  将NURBS离散为密集点列
                └────────────────┘
                        │
                     	▼
         ───────────────────────────────┐
         │                               │
         ▼                               ▼
┌─────────────────┐           ┌─────────────────┐
│  全局拟合模块     │           │  局部光顺模块     │
│  (路径拟合)      │           │  (路径光顺)       │
└────────┬────────┘           └────────┬────────┘
         │                               │
         └───────────┬───────────────────┘
                     │
                     ▼
            ┌─────────────────┐
            │  速度规划模块     │  根据曲率/弦差计算速度
            └────────┬────────┘
                     │
                     ▼
            ┌─────────────────┐
            │  G代码生成模块    │  输出可执行G代码
            └────────┬────────┘
                     │
                     ▼
                  G代码
```

### 4.2 流程配置

```cpp
// 路径处理模式枚举 (同级选择, 二选一)
enum class PathProcessingMode {
    GlobalFitting,   // 全局拟合模式: 将离散点拟合为NURBS曲线
    LocalSmoothing   // 局部光顺模式: 对离散路径进行平滑处理
};

// 流程步骤枚举
enum class ProcessStep {
    Discretization,
    PathProcessing,   // 路径处理(包含拟合/光顺同级选择)
    SpeedPlanning,
    GCodeGeneration
};

// 流程配置
struct PipelineConfig {
    // 路径处理模式选择 (同级选择, 二选一)
    PathProcessingMode path_processing_mode = PathProcessingMode::LocalSmoothing;

    // 各步骤是否启用
    bool enable_discretization = true;
    bool enable_speed_planning = true;

    // 各步骤算法选择
    std::string discretization_method = "Adaptive_Chordal";

    // 全局拟合算法配置 (当 mode == GlobalFitting 时使用)
    struct GlobalFittingConfig {
        bool enabled = false;
        std::string algorithm = "G2_Spline";
        IGlobalFittingAlgorithm::Parameters params;
    };

    // 局部光顺算法配置 (当 mode == LocalSmoothing 时使用)
    struct LocalSmoothingConfig {
        bool enabled = true;
        std::string algorithm = "Parametric_Spline";
        ILocalSmoothingAlgorithm::Parameters params;
    };

    GlobalFittingConfig global_fitting;
    LocalSmoothingConfig local_smoothing;

    std::string speed_control_algorithm = "Chordal_Error";
    ISpeedControlAlgorithm::Parameters speed_params;
};

// 流程执行器
class ProcessingPipeline {
public:
    explicit ProcessingPipeline(const PipelineConfig& config);

    // 执行完整流程
    GCodeProgram execute(const NURBSCurve& nurbs);
    GCodeProgram execute(const DiscretizedPath& path);

    // 分步执行(用于调试)
    DiscretizedPath stepDiscretize(const NURBSCurve& nurbs);
    DiscretizedPath stepPathProcess(const DiscretizedPath& path);  // 统一路径处理入口
    DiscretizedPath stepSpeedPlan(const DiscretizedPath& path);
    GCodeProgram stepGenerate(const DiscretizedPath& path);

    // 获取中间结果
    const DiscretizedPath& getDiscretizedPath() const;
    const DiscretizedPath& getProcessedPath() const;  // 拟合或光顺后的路径

private:
    PipelineConfig config_;
    // 中间结果缓存
    DiscretizedPath discretized_path_;
    DiscretizedPath processed_path_;    // 全局拟合或局部光顺的结果
    DiscretizedPath speed_planned_path_;
};
```

---

## 5. 五轴机床预留扩展

### 5.1 五轴运动学模型

```cpp
// 五轴机床基类
class IFiveAxisMachine {
public:
    virtual ~IFiveAxisMachine() = default;

    // 运动学类型
    enum class KinematicsType {
        TableTable,     // 双转台
        TableTool,      // 转台+摆头
        DualTable       // 双摆台
    };

    virtual KinematicsType type() const = 0;

    // 正逆运动学
    // 工件坐标系 -> 机床坐标系
    virtual ToolPose5Axis forward(
        double x, double y, double z,
        double a, double b
    ) const = 0;

    // 机床坐标系 -> 工件坐标系
    virtual std::tuple<double, double, double, double, double> inverse(
        const Point3D& position,
        const Vec3D& direction
    ) const = 0;

    // 边界检查
    virtual bool isWithinLimits(
        double x, double y, double z,
        double a, double b
    ) const = 0;

    // 奇异点检测
    virtual bool isSingular(double a, double b) const = 0;
};

// 五轴机床实现示例(转台+摆头)
class FiveAxisTableTool : public IFiveAxisMachine {
public:
    KinematicsType type() const override {
        return KinematicsType::TableTool;
    }

    // 工件坐标系到机床坐标系
    ToolPose5Axis forward(double x, double y, double z,
                          double a, double b) const override {
        // 1. 应用工件装夹偏置
        // 2. 应用刀具长度补偿
        // 3. 应用旋转轴角度
        // 4. 得到机床实际位置
    }

    // 机床坐标系到工件坐标系
    std::tuple<double, double, double, double, double> inverse(
        const Point3D& position,
        const Vec3D& direction
    ) const override {
        // 逆运动学求解
    }

    bool isWithinLimits(double x, double y, double z,
                        double a, double b) const override {
        // 轴行程检查
    }

    bool isSingular(double a, double b) const override {
        // B轴在0°或90°时为奇异点
        return std::abs(std::cos(b)) < 0.01;
    }
};
```

### 5.2 五轴刀位规划

```cpp
// 五轴刀位数据
struct FiveAxisCLData {
    Point3D tool_tip;           // 刀尖点位置
    Vec3D tool_axis;            // 刀轴方向
    Vec3D surface_normal;       // 加工面法向(用于刀轴控制)
    double feedrate;
    double arc_length;
};

// 五轴刀位规划器(抽象接口)
class IFiveAxisCLPlanner {
public:
    virtual ~IFiveAxisCLPlanner() = default;

    // 从三轴刀位扩展到五轴
    virtual std::vector<FiveAxisCLData> planCL(
        const std::vector<Point3D>& three_axis_cl,
        const ToolPose5Axis::MachinePosition& home_position,
        const IFiveAxisMachine& machine
    ) = 0;

    // 刀轴控制策略
    enum class ToolAxisStrategy {
        Normal,             // 垂直于表面
        Tilted,             // 倾斜一定角度
        LeadLag,            // 前倾/后倾
        SideTilt            // 侧倾
    };

    virtual void setToolAxisStrategy(ToolAxisStrategy strategy) = 0;
};

// 五轴后处理器(将刀位转为G代码)
class FiveAxisPostProcessor {
public:
    explicit FiveAxisPostProcessor(const IFiveAxisMachine& machine);

    // 生成五轴G代码
    virtual std::vector<GCodeCommand> generateGCode(
        const std::vector<FiveAxisCLData>& cl_data,
        const MachineContext& context
    ) = 0;

    // 特殊G代码处理
    virtual GCodeCommand formatLinearMove(
        const FiveAxisCLData& cl,
        bool use_abc_axes = false  // 选择转轴组合
    ) = 0;

    virtual GCodeCommand formatCircularMove(
        const FiveAxisCLData& start,
        const FiveAxisCLData& end,
        const Point3D& center
    ) = 0;
};
```

---

## 6. 可视化模块

### 6.1 OpenGL窗口结构

```cpp
// 主可视化窗口
class VisualizationWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit VisualizationWidget(QWidget* parent = nullptr);
    ~VisualizationWidget() override;

    // 数据绑定
    void setDiscretizedPath(const DiscretizedPath* path);
    void setSmoothedPath(const DiscretizedPath* path);
    void setGCodeProgram(const GCodeProgram* program);
    void setMachineContext(const MachineContext* context);

    // 刀具动画
    void startAnimation();
    void stopAnimation();
    void setAnimationSpeed(double speed);  // 倍率

    // 视角控制
    enum class ViewMode { Top, Front, Side, Iso, Free };
    void setViewMode(ViewMode mode);

signals:
    void animationProgressChanged(double progress);  // 0~1

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void setupShaders();
    void renderGrid();
    void renderPath();
    void renderTool();
    void renderMachine();

    // 数据指针(不拥有)
    const DiscretizedPath* path_to_show_ = nullptr;
    const DiscretizedPath* smoothed_path_ = nullptr;
    const GCodeProgram* gcode_program_ = nullptr;
    const MachineContext* machine_context_ = nullptr;

    // OpenGL资源
    std::unique_ptr<QOpenGLShaderProgram> path_shader_;
    std::unique_ptr<QOpenGLShaderProgram> tool_shader_;
    std::unique_ptr<QOpenGLShaderProgram> grid_shader_;

    // 渲染器
    PathRenderer path_renderer_;
    ToolRenderer tool_renderer_;
    GridRenderer grid_renderer_;

    // 相机
    Camera camera_;
    ViewMode view_mode_;

    // 动画
    double animation_time_ = 0.0;
    double animation_speed_ = 1.0;
    bool animating_ = false;
};
```

### 6.2 路径渲染器

```cpp
class PathRenderer {
public:
    PathRenderer();

    // 上传路径数据到GPU
    void uploadPath(const DiscretizedPath& path);

    // 颜色配置
    void setOriginalColor(const QColor& color);
    void setSmoothedColor(const QColor& color);
    void setPlannedColor(const QColor& color);

    // 速度着色
    void enableSpeedColoring(bool enable);
    void setSpeedRange(double min, double max);

    // 渲染
    void render(RenderPass pass);

private:
    void updateVertexBuffers();

    // VBO数据
    std::vector<float> vertices_;     // 位置 + 颜色
    std::vector<unsigned int> indices_;

    // 速度颜色映射
    bool speed_coloring_enabled_ = false;
    QColor min_speed_color_;   // 蓝色(低速)
    QColor max_speed_color_;   // 红色(高速)
};
```

### 6.3 刀具渲染器

```cpp
class ToolRenderer {
public:
    ToolRenderer();

    // 刀具几何
    void setToolGeometry(const ToolGeometry& geo);

    // 位姿更新
    void updatePose(const ToolPose& pose);

    // 轨迹历史
    void setTrailLength(int max_points);
    void pushTrailPoint(const Point3D& point);

    // 渲染模式
    enum class RenderMode {
        Solid,         // 实体渲染
        Wireframe,     // 线框渲染
        Transparent    // 半透明
    };
    void setRenderMode(RenderMode mode);

    void render();

private:
    ToolGeometry geometry_;
    ToolPose current_pose_;
    std::vector<Point3D> trail_;
    int max_trail_points_ = 1000;

    RenderMode render_mode_ = RenderMode::Solid;
};
```

### 6.4 相机控制

```cpp
class Camera {
public:
    Camera();

    // 视角设置
    void setTopView();
    void setFrontView();
    void setSideView();
    void setIsometricView();

    // 自由视角
    void orbit(double dx, double dy);    // 旋转
    void pan(double dx, double dy);     // 平移
    void zoom(double factor);           // 缩放

    // 焦点跟踪
    void setTarget(const Point3D& target);

    // 矩阵获取
    QMatrix4x4 viewMatrix() const;
    QMatrix4x4 projectionMatrix() const;

private:
    Point3D position_;
    Point3D target_;
    Vec3D up_vector_;

    double fov_ = 45.0;
    double near_plane_ = 0.1;
    double far_plane_ = 1000.0;

    ViewMode mode_;
};
```

---

## 7. 数据流与信号槽

### 7.1 核心信号

```cpp
// 算法执行信号
signals:
    void fittingStarted(const QString& algorithm);
    void fittingFinished(const NURBSCurve& result);
    void fittingProgress(int percent);

    void smoothingStarted(const QString& algorithm);
    void smoothingFinished(const DiscretizedPath& result);
    void smoothingProgress(int percent);

    void speedPlanningStarted(const QString& algorithm);
    void speedPlanningFinished(const DiscretizedPath& result);

    void gcodeGenerationStarted();
    void gcodeGenerationFinished(const GCodeProgram& result);

// 可视化信号
signals:
    void pathUpdated(const DiscretizedPath* path);
    void animationFrameUpdated(double time);
    void cameraViewChanged(ViewMode mode);
```

### 7.2 主要类协作

```
┌──────────────────┐     ┌──────────────────┐
│  MainWindow      │     │  ProcessingPanel │
└────────┬─────────┘     └────────┬─────────┘
         │                        │
         │  setConfig()           │  execute()
         ▼                        ▼
┌──────────────────────────────────────────────┐
│              ProcessingPipeline              │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ │
│  │Discret│→│Fitting │→│Smoothing│→│ Speed │→│GCode│
│  └────────┘ └────────┘ └────────┘ └────────┘ │
└──────────────────────────────────────────────┘
         │              │
         │ setPath()    │ update()
         ▼              ▼
┌──────────────────┐  ┌──────────────────┐
│VisualizationWidget│  │  ResultsPanel    │
└──────────────────┘  └──────────────────┘
```

---

## 8. 接口扩展指南

### 8.1 新增全局拟合算法

```cpp
// 1. 继承基类
class MyFittingAlgorithm : public IGlobalFittingAlgorithm {
public:
    std::string name() const override { return "My_Fitting"; }

    NURBSCurve fit(const DiscretizedPath& path) override {
        // 实现拟合逻辑
    }
};

// 2. 注册算法(放在static变量或初始化函数中)
REGISTER_FITTING_ALGORITHM("My_Fitting", MyFittingAlgorithm);

// 3. 在UI中使用
auto algorithm = GlobalFittingFactory::instance().create("My_Fitting");
algorithm->setParameters(my_params);
NURBSCurve result = algorithm->fit(input_path);
```

### 8.2 新增局部光顺算法

```cpp
class MySmoothingAlgorithm : public ILocalSmoothingAlgorithm {
public:
    std::string name() const override { return "My_Smoothing"; }

    DiscretizedPath smooth(const DiscretizedPath& input) override {
        // 实现光顺逻辑
    }
};

REGISTER_SMOOTHING_ALGORITHM("My_Smoothing", MySmoothingAlgorithm);
```

### 8.3 新增速度控制算法

```cpp
class MySpeedAlgorithm : public ISpeedControlAlgorithm {
public:
    std::string name() const override { return "My_Speed"; }

    DiscretizedPath planSpeed(const DiscretizedPath& path,
                              const MachineContext& machine) override {
        // 实现速度规划逻辑
    }
};

REGISTER_SPEED_ALGORITHM("My_Speed", MySpeedAlgorithm);
```

---

## 9. 关键算法参数配置

### 9.1 推荐参数范围

| 参数类别 | 参数名             | 推荐值          | 说明         |
| -------- | ------------------ | --------------- | ------------ |
| 离散化   | chord_error        | 0.001~0.01mm    | 弦高误差     |
| 离散化   | max_segment_length | 0.5~2mm         | 最大段长     |
| 全局拟合 | tolerance          | 0.001~0.01mm    | 拟合公差     |
| 全局拟合 | max_control_points | 30~100          | 最大控制点数 |
| 局部光顺 | smoothing_weight   | 0.3~0.7         | 光顺权重     |
| 局部光顺 | window_size        | 5~15            | 平滑窗口     |
| 速度控制 | max_feedrate       | 3000~8000mm/min | 最大进给     |
| 速度控制 | acceleration       | 200~1000mm/s²   | 最大加速度   |

### 9.2 参数继承关系

```
G代码公差
    │
    ├── chord_error (离散化) ≤ G代码公差 × 0.5
    │
    ├── fitting_tolerance (全局拟合) ≤ G代码公差
    │
    ├── smoothing_chord_error (光顺) ≤ chord_error
    │
    └── speed_chord_error (速度规划) ≤ chord_error
```

---

## 10. 技术总结

### 10.1 核心接口契约

| 接口                     | 输入                             | 输出                        | 可扩展性     |
| ------------------------ | -------------------------------- | --------------------------- | ------------ |
| IGlobalFittingAlgorithm  | DiscretizedPath                  | NURBSCurve                  | 完全可扩展   |
| ILocalSmoothingAlgorithm | DiscretizedPath                  | DiscretizedPath             | 完全可扩展   |
| ISpeedControlAlgorithm   | DiscretizedPath + MachineContext | DiscretizedPath(with speed) | 完全可扩展   |
| IFiveAxisMachine         | XYZ + ABC                        | ToolPose5Axis               | 可替换实现   |
| FiveAxisPostProcessor    | FiveAxisCLData                   | GCodeCommand                | 可扩展后处理 |

### 10.2 插件化支持

通过工厂模式和注册宏,新算法可以在不修改框架代码的情况下加入:

- 编译时插件: 直接注册到工厂
- 运行时插件: 通过动态库加载(可选扩展)

### 10.3 扩展性保障

1. **算法隔离**: 各算法独立,不相互依赖
2. **数据统一**: 所有算法使用统一的路径格式
3. **工厂解耦**: 算法创建与使用分离
4. **五轴预留**: ToolPose基类已包含五轴扩展,不影响三轴实现
