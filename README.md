# Calibrate_and_Measure

印刷品尺寸测量 C++ 工程：棋盘格制板、相机标定、图像测量（宽 / 高 / 上半部分
水平边长）。算法由 Python 版 `D:/project/python_release/test_edge_detect`
等价移植。本文档是唯一使用说明，看完即可接入。

- 软件同事接入：只需包含一个头文件 `src/cam_api.h`，命名空间 `cam`。
  本文第 1~7 节是完整调用约定。
- 工程维护者自用：第 8 节起是配置、依赖、目录结构与调试入口。

## 1. 一分钟接入

```cpp
#include "cam_api.h"   // src/ 目录下唯一需要包含的头文件

// 软件启动时：初始化一次（重操作，秒级）
cam::Measurer measurer;
std::string err;
if (!measurer.Init("config.ini", err)) {        // ini 路径显式传入
    // err 为中文失败原因
}

// 每收到一帧相机原图：调用一次
cam::MeasureOutput out = measurer.Measure(frame, "tag");  // frame 为 cv::Mat
if (out.code == cam::RetCode::OK) {
    double wPx = out.width.widthPx;             // 宽度（px）
    double hPx = out.height.heightPx;           // 高度（px）
    // 几何矫正开启时换算毫米：mm = px × MmPerPx()
    double wMm = wPx * measurer.MmPerPx();
    // out.horizontalEdges 为上半部分候选水平边列表
} else {
    // out.message 为中文失败原因
}
```

制板与标定同样走门面：`cam::MakeBoard(iniPath, err)`、
`cam::Calibrate(images, iniPath, &report, err)`（返回 `CalibReport` 质量摘要）。

## 2. 接口总览（`src/cam_api.h`，命名空间 `cam`）

| 接口 | 用途 | 调用频率 |
|---|---|---|
| `Version()` | 门面版本串 | 任意 |
| `MakeBoard(iniPath, errMsg)` | 功能 1：生成标定板 PDF + 预览图 + 打印说明 | 按需 |
| `Calibrate(images, iniPath, report, errMsg)` | 功能 2：标定求解，1~n 张采图，返回 `CalibReport` | 换机位/换板时一次 |
| `Measurer::Init(iniPath, errMsg)` | 功能 3 初始化：配置 + 背景建模 + ONNX 会话预热 + 可选矫正 | 启动时一次 |
| `Measurer::Measure(image, debugTag)` | 功能 3 单帧测量：矫正（内部）→ 分割 → 旋转校正 → 宽高与水平边 | 逐帧 |
| `Measurer::MmPerPx()` / `RectifyEnabled()` / `UsingAi()` | 运行形态查询与毫米换算 | 任意 |

## 3. 共同约定（三个功能都适用）

- 配置：所有参数集中在一份 ini 文件（工程自带 `config.ini`）。每个接口都以
  iniPath 为配置入口，调用方把这份 ini 随软件分发即可，改参数不用重编译。
  ini 内的相对路径相对 ini 文件所在目录解析，ini 可放任意位置。
- 错误处理：功能 1/2 与 `Measurer::Init` 返回 `bool` + 输出参数 `errMsg`
  （中文原因）；逐帧测量返回 `MeasureOutput`，其中 `code` 为返回码、
  `message` 为中文原因。接口内部已做异常兜底，不会向外抛异常；建议调用方
  仍在外层加 try/catch 保险。
- 日志：运行过程统一经 `common::LogMsg` 输出中文分级日志（Debug/Info/Warn/
  Error），集成时可在 `common/logger.h` 处改挂到自己的日志系统。
- 线程：`Measurer` 非线程安全，请串行调用；多相机场景每相机一个实例。
- 中间结果：ini 的 `[debug] save_intermediate` 为总开关。true 时标定出 QA
  质检图、测量落过程图（见 6.2）；false 为部署模式，只产出最终结果。

## 4. 功能 1：生成棋盘格标定板打印文件

```cpp
bool cam::MakeBoard(const std::string& iniPath, std::string& errMsg);
```

- 入参只有 iniPath，全部版面参数读 ini 的 `[checkerboard]` 段：
  `square_mm`（格边长 mm）、`cols`/`rows`（格数）、`border_mm`（留白）、
  `paper_w_mm`/`paper_h_mm`（纸张尺寸）、`out_dir`（产物目录）。
- 产物三件，落在 `[checkerboard] out_dir`：
  ▸ `checkerboard.pdf` 矢量打印文件（交打印店，100% 实际大小打印）
  ▸ `checkerboard_preview.png` 人工核对预览图（勿打印）
  ▸ `打印说明.txt` 随参数自动生成的打印要求说明
- 返回 false 时 errMsg 为中文原因（参数非法，如边距不足、格数奇偶相同；
  或写盘失败）。

## 5. 功能 2：棋盘格标定（生成标定文件）

```cpp
struct cam::CalibReport {          // 标定摘要（机器可读）
    double      rms;               // RMS 重投影误差（px）
    double      verifyMeanPx;      // 正射验证闭环平均残差（px）
    double      verifyP95Px;       // 正射验证闭环 p95 残差（px）
    std::string xmlPath;           // 标定文件实际输出路径
};

bool cam::Calibrate(const std::vector<cv::Mat>& boardImages,
                    const std::string& iniPath,
                    cam::CalibReport* report, std::string& errMsg);
```

- 传入 1~n 张采图即可，门面自动区分单图/多图（多图固定机位连拍，角点
  取均值降噪，更稳）。`report` 可传 nullptr 忽略。
- 输入图像支持 8/24/32 通道（内部自动转灰度），多张尺寸须一致。
- 参数读 ini 的 `[calibrate]` 段：`pattern_cols`/`pattern_rows`（内角点 =
  格数-1）、`square_x_mm`/`square_y_mm`（实测格距，务必用卡尺量打印实物后
  回填）、`out_xml`（标定文件输出路径）、`target_mm_per_px`（正射刻度）、
  `qa_dir`（QA 图目录）。
- 产物：标定文件 `[calibrate] out_xml`（OpenCV XML：内参/畸变/外参/格距/
  正射刻度）；`[debug]` 打开时另有 4 张 QA 质检图落 `qa_dir`。
- 质量门禁：RMS > 0.3px 或校正验证 mean > 0.5px 时返回 false，但文件与
  QA 图照常输出，`report` 内的已知数值照常填写（验证未完成时 verify* 为 0），
  errMsg 说明质量偏低。可据 QA 图检查印刷精度与采图条件后重标。

## 6. 功能 3：单张图像测量

调用模型：`Measurer` 对象 Init 一次，之后每帧相机原图调一次 `Measure`。

### 6.1 初始化（软件启动时一次）

```cpp
cam::Measurer measurer;
std::string err;
if (!measurer.Init(iniPath, err)) {
    // err 为中文原因（配置打不开、背景建模失败、矫正标定文件缺失等）
}

// 初始化后可查询实际生效的运行形态（建议上位机展示或记日志）：
measurer.RectifyEnabled();  // 几何矫正是否生效（[rectify] enabled 且标定加载成功）
measurer.MmPerPx();         // 毫米换算系数；矫正关闭时返回 0
measurer.UsingAi();         // 实际生效的分割是否 AI 链（回退传统后为 false）
```

`Init` 是重操作，只做一次，内部依次完成：

1. 加载 ini 全量配置。
2. 背景建模：`[paths] background_file` 缓存优先；无缓存时用
   `[paths] input_dir` 全量图现建并写缓存。交付现场可随包分发预生成的
   缓存文件，免放全量输入图。
3. 分割器：`[segmentation] method=="ai"` 时创建 ONNX 会话并完成预热；
   模型缺失/加载失败自动回退传统背景差分并记 Warn，Init 仍成功
   （用 `UsingAi()` 确认实际链路）。
4. `[rectify] enabled=true` 时加载标定 XML 构建正射 remap 表，加载失败
   Init 返回 false。矫正开启时 Init 还会用同一标定文件把背景模型同步
   正射校正，调用方无需处理。

深度学习模型与预热的细节：

- AI 分割使用 ONNX Runtime 推理 `assets/weights/small.onnx`（RF-DETR-seg
  small，Python 侧导出）。`[ai_seg] device`：`auto`（默认，预检 CUDA 运行时，
  缺失静默走 CPU）/ `cuda`（强制 GPU）/ `cpu`。
- 会话创建成功后自动做一次预热推理，Init 返回时预热已完成，调用方无需做
  任何事，也没有配置开关。预热的具体动作：用一张模型输入尺寸的合成纯灰图
  （灰度 128，尺寸跟随模型实际输入，如 384x384）跑一次完整推理，结果直接
  丢弃。计算图执行与图像内容无关，合成图足以触发全部首次开销。
- 预热覆盖首次推理慢的三大来源：cuDNN 卷积策略搜索、显存分配器扩张、
  CUDA 模块加载。不预热的话，现场第一帧测量会出现秒级卡顿。
- 如何确认生效：初始化日志出现"模型预热完成，首次推理耗时 xx ms"即成功；
  若预热抛异常，只打 Warn"模型预热失败（不影响后续推理）"，分割器仍为就绪。
- CPU 模式下预热同样执行，开销小、无害；纯灰图无产品检出属预期现象，
  InferCrop 返回 false 不算预热失败。

### 6.2 逐帧测量（每收到一帧相机原图）

```cpp
cv::Mat raw = /* 相机原始图，8/24/32 通道均可 */;

cam::MeasureOutput out = measurer.Measure(raw, "frame_tag");  // debugTag 可传 ""
if (out.code == cam::RetCode::OK) {
    // out.width.widthPx / out.height.heightPx / out.horizontalEdges 有效
    double widthMm = out.width.widthPx * measurer.MmPerPx();  // 矫正开启时
} else {
    // out.message 为中文失败原因
}
```

- 矫正（若开启）由 `Measure` 内部完成：原图 → 转灰度 → 正射矫正 → 测量。
  调用方只传相机原图，不需要也不应该自己先调 Rectifier。
- debugTag 仅在 `[debug] save_intermediate=true` 时生效，非空则向
  `[paths] output_dir/debug/<debugTag>/` 落 6 张过程图（输入灰度/分割掩膜/
  增强图/旋转图/旋转掩膜/测量叠加图），便于部署现场排查；传 "" 则零中间文件。
- 未初始化（Init 未调用或失败）时调用 Measure，返回 `code=INTERNAL`，
  message 提示先初始化。
- 像素换算：输出均为像素值。矫正开启时，毫米 = 像素 × `MmPerPx()`
  （默认刻度 0.15 mm/px）。

### 6.3 输出数据结构（完整定义，与 `src/measure_types.h` 逐行一致）

坐标系约定：角度校正后的图像坐标系，原点左上，x 向右，y 向下。
单位全部为像素（px），浮点，亚像素保留 2 位小数；角度保留 3 位小数。

```cpp
// 返回码
enum class RetCode : int {
    OK          = 0,    // 成功
    EMPTY_IMAGE = 1,    // 输入图像为空
    BAD_FORMAT  = 2,    // 图像格式不支持（须 8UC1/8UC3/8UC4）
    NO_PRODUCT  = 3,    // 未检出产品
    INTERNAL    = 100   // 算法内部异常（message 附中文说明）
};

struct Point2d {                     // 像素坐标点（亚像素）
    double x;
    double y;
};

struct LineSegment {                 // 有限线段
    Point2d start;                   // 起点
    Point2d end;                     // 终点
};

struct WidthResult {                 // 结构体一：产品宽度
    LineSegment leftLine;            // 左侧竖线起点/终点
    LineSegment rightLine;           // 右侧竖线起点/终点
    double      widthPx;             // 宽度数值（左右竖线间距）
    bool        ok;
};

struct HeightResult {                // 结构体二：产品高度
    LineSegment topLine;             // 上边横线起点/终点
    LineSegment bottomLine;          // 下边横线起点/终点
    double      heightPx;            // 高度数值（上下横线间距）
    bool        ok;
};

struct HorizontalEdge {              // 结构体三：上半部分一条水平边线段
    Point2d start;                   // 起点（亚像素，拟合直线上的投影点）
    Point2d end;                     // 终点（亚像素）
    double  lengthPx;                // 线段长度（px，沿拟合直线的跨度）
    bool    satisfied;               // 是否通过 [measure] edge_width_threshold 宽度判定
};

struct MeasureOutput {               // 单帧完整测量结果（Measure 的返回值）
    RetCode     code;                // 返回码
    std::string message;             // 失败时的中文原因，成功为空
    double      rotationDeg;         // 总校正角（度）
    WidthResult  width;              // 宽度（code==OK 时有效）
    HeightResult height;             // 高度（code==OK 时有效）
    std::vector<HorizontalEdge> horizontalEdges;
                                     // 上半部分候选水平边，有多少条返回多少条，
                                     // 按 (y, x_left) 排序
};
```

宽高四线端点约定：四线围成闭合测量矩形。左/右竖线的纵向范围就是上/下横线的
y 位置，上/下横线的横向范围就是左/右竖线的 x 位置：

```text
              topLine（高度上边）
        (x_l,y_t) ────────────── (x_r,y_t)
           │                        │
       leftLine                 rightLine
      （宽度左竖线）           （宽度右竖线）
           │                        │
        (x_l,y_b) ────────────── (x_r,y_b)
             bottomLine（高度下边）

   widthPx = x_r - x_l     heightPx = y_b - y_t
```

一次典型测量的填充示例（数值仅为示意）：

```text
code = OK,  message = "",  rotationDeg = 1.283

width.widthPx = 812.45
width.leftLine  = (320.15, 410.22) - (320.87, 1450.61)
width.rightLine = (1132.60, 410.22) - (1133.05, 1450.61)

height.heightPx = 1040.39
height.topLine    = (320.15, 410.22) - (1132.60, 410.22)
height.bottomLine = (320.87, 1450.61) - (1133.05, 1450.61)

horizontalEdges（2 条）:
  [0] start=(352.10, 462.30) end=(598.44, 463.02) lengthPx=246.36 satisfied=1
  [1] start=(700.05, 462.55) end=(913.77, 463.10) lengthPx=213.73 satisfied=0
```

## 7. 集成注意事项

- 三个功能的菜单演示在 `main.cpp`，批量测量与 CSV 汇总也仅是演示代码；
  集成时按第 6 节的单帧流程对接即可。
- 标定采图、背景建模、测量采图三者图像尺寸必须一致（同一相机同一分辨率）。
- 测量相关 ini 段速查：`[rectify]` 矫正开关与标定文件；`[paths]` 输入输出与
  背景缓存；`[segmentation] method` 选 `ai`（默认）或 `traditional`；
  `[trad_seg]` / `[refine]` / `[rotate]` / `[measure]` 为各算法参数，
  默认值与 Python 版 `configs/default.yaml` 对齐。

## 8. 配置文件

- 全部参数集中在 `config.ini`，按功能分段：`[checkerboard]` 制板、
  `[calibrate]` 标定、`[rectify]` 矫正开关、`[paths]` 输入输出、
  `[segmentation]`/`[ai_seg]`/`[trad_seg]` 分割、`[refine]`/`[rotate]`/
  `[measure]` 算法参数、`[debug]` 中间结果开关。各键中文注释见文件内。
- 接口以 ini 路径为配置入口，显式传参；ini 内相对路径相对 ini 文件所在
  目录解析，ini 可随软件放任意位置。
- 默认值与 Python 版 `configs/default.yaml` 对齐。

## 9. 依赖与分发清单

- 开发/编译：Visual Studio 2019（v142）+ C++17；OpenCV 4.8.0；
  ONNX Runtime GPU 版 1.20.1。第三方库路径统一在 `opencv_onnx_paths.props`
  里改（唯一需要手改的文件）。
- 随软件分发的运行时 dll：
  ▸ `opencv_world480.dll`
  ▸ `onnxruntime.dll`（AI 分割必需）
  ▸ `onnxruntime_providers_cuda.dll` 与 `onnxruntime_providers_shared.dll`
    （仅 GPU 模式需要）
- GPU 模式的部署机还需 CUDA 12.x + cuDNN 9.x 运行时；纯 CPU 模式零 CUDA 依赖。
- 数据文件：`config.ini`、`assets/weights/small.onnx`（AI 模式）、
  标定产物 `checkerboard_calib.xml`（矫正开启时）、背景缓存
  `background_model.bmp`（可预生成随包分发，免去现场建模）。

## 10. 目录结构

```
Calibrate_and_Measure/
├── Calibrate_and_Measure.sln / .vcxproj   # VS2019 (v142) 工程
├── opencv_onnx_paths.props                # ★ 第三方库路径（唯一需要手动改的文件）
├── config.ini                             # ★ 全工程配置（按功能分段，中文注释）
├── main.cpp                               # 菜单驱动 demo（自用调试入口）
├── src/
│   ├── cam_api.h / cam_api.cpp            # ★ 对外统一门面（集成唯一入口）
│   ├── measure_types.h                    # 对外结构体（MeasureOutput 等）
│   ├── common/                            # ini 解析 / 图像 IO（8/24/32 位 BMP 兜底）/ 日志
│   ├── checkerboard/                      # 功能 1：制板
│   ├── calibration/                       # 功能 2：标定求解 + 正射矫正器
│   └── measure/                           # 功能 3：背景建模 / 分割（AI+传统）/ 精修 / 校正 / 测量
├── assets/                                # 制板产物、标定 XML、ONNX 模型、测试数据
│   └── test_data/                         # calibration/ 标定采图、input/ 待测图、output/ 结果
├── docs/design/                           # 设计文档（含三大功能流程图，Mermaid + PNG）
├── temp/                                  # 中间产物（QA 图、背景缓存）
```

## 11. 自用调试入口（demo）

运行 `Calibrate_and_Measure.exe` 进入菜单（配置固定取工程根 `config.ini`）：

1. `1` 生成标定板 → 把 `assets/checkerboard/打印说明.txt` 和 PDF 交打印店
   （100% 实际大小打印）
2. `2` 打印板铺在吸风展平板上采图放入 `assets/test_data/calibration/` 执行标定
   → 生成 `assets/calibration/checkerboard_calib.xml` + QA 图
   （`temp/calibration_qa/`）；也可把单张采图直接拖到 exe 上快捷标定；
   用卡尺实测打印格距，回填 `config.ini` 的 `[calibrate] square_x_mm/square_y_mm`
3. 标定达标后把 `[rectify] enabled` 改为 `true`
4. 待测产品图放入 `assets/test_data/input/`，菜单选 `3` 批量测量：控制台逐张
   打印结果，汇总写 `assets/test_data/output/measure_results.csv`（含宽高四线
   端点坐标），每条候选水平边写 `measure_edges.csv`；`[debug]` 打开时过程图落
   `assets/test_data/output/debug/<图像名>/`

## 12. 已知说明

- 标定质量门禁：RMS ≤ 0.3px 且校正验证 mean ≤ 0.5px，越限时产物保留但返回
  失败并给中文原因（`CalibReport` 内数值照常填写）。
- AI 分割模型为 `assets/weights/small.onnx`（Python 侧 RF-DETR-seg small 导出）。
- RANSAC 随机源为 `cv::RNG(42)`，与 Python numpy PCG64(42) 统计语义等价但样本
  不同，与 Python 对拍可能存在亚像素级差异。
- 编译时 main.cpp 可能出现 C4819/C4477 警告（MSVC 对 UTF-8 中文字面量 printf
  格式检查的误报），不影响功能。

## 变更记录

- 2026-09-14（2）：INTERFACE.md 并入 README，本文档成为唯一使用说明。
- 2026-09-14：交付化封装。新增统一门面 `src/cam_api.h/.cpp`
  （`MakeBoard`/`Calibrate`/`Measurer`/`Version`）；几何矫正内收到
  `Measurer::Measure` 内部；标定新增 `CalibReport` 机器可读摘要；ini 相对
  路径改按 ini 所在目录解析（不再依赖 exe 目录层级）；`MeasurePipeline` 新增
  `UsingAi()`；`main.cpp` 重写为纯门面 demo。
- 2026-09-09：新增 `docs/design/2026-09-09-三大功能流程图.md`，制板 / 标定 /
  单帧测量三张 Mermaid 流程图（与代码逐模块核对），附 mmdc 渲染的 PNG 供无
  Mermaid 环境直接查看。
- 2026-09-08（4）：AI 分割会话创建后内置模型预热（合成灰图跑通一次完整推理，
  消除现场首帧秒级卡顿；不加配置开关，默认即做，预热失败仅记 Warn 不影响就绪）。
- 2026-09-08（3）：新增 `INTERFACE.md` 集成调用接口说明（制板 / 标定 / 单帧
  测量，含 MeasureOutput 字段表与示例代码）。
- 2026-09-08（2）：`test_data/` 并入 `assets/`；`[ai_seg] device` 新增 `auto`
  （默认）：预检 CUDA 运行时可用才注册 CUDA EP，缺运行时静默走 CPU；新增
  `[debug] save_intermediate` 总开关；测量 CSV 拆为 `measure_results.csv`
  （汇总）+ `measure_edges.csv`（边明细）。
- 2026-09-08：初版建立。自 FastBilateralNPP 工程改名重建；完成制板 / 标定 /
  矫正 / 测量（传统链 + AI 接口）全模块移植。OpenCV 4.8.0 与 ONNX Runtime GPU
  1.20.1 路径已配入 props；工程不直接链接 CUDA（GPU 推理由 ORT CUDA EP 承担，
  部署机需 CUDA 12 + cuDNN 9 运行时）。
