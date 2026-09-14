# Calibrate_and_Measure 调用接口说明

面向集成方（上位机软件）的三大功能调用约定。工程内 `main.cpp` 的菜单只是演示壳，
集成时按下文直接调用 `cam` 命名空间下的接口即可。

- 2026-09-08 编写，与当日代码版本一致。
- 全部接口位于 `src/` 下公开头文件，命名空间 `cam`。
- 数据类型约定见 `src/measure_types.h`，与 Python 版 `INTERFACE.md` v1.4 对齐。

## 0. 共同约定（三个功能都适用）

- 配置：所有参数集中在一份 ini 文件（工程自带 `config.ini`）。每个接口的第一个
  或第二个参数都是 iniPath，调用方把这份 ini 随软件分发即可，改参数不用重编译。
- 错误处理：功能 1/2 返回 `bool` + 输出参数 `errMsg`（中文原因）；功能 3 返回
  `MeasureOutput`，其中 `code` 为返回码、`message` 为中文原因。接口内部已做异常
  兜底，不会向外抛异常；建议调用方仍在外层加 try/catch 保险。
- 日志：运行过程统一经 `common::LogMsg` 输出中文分级日志（Debug/Info/Warn/Error），
  集成时可在 `common/logger.h` 处改挂到自己的日志系统。
- 路径解析：ini 里的相对路径相对"工程根目录"解析。工程根 = 调用进程 exe 所在目录
  的上两级；该目录下找不到 config.ini 时退回 exe 同目录。集成到自己软件时，exe 目录
  结构不同，建议 ini 中直接写绝对路径，最稳妥。
- 线程：各模块未做并发调用验证，请串行调用（Init/Load 一次，逐帧 Measure）。
- 中间结果：ini 的 `[debug] save_intermediate` 为总开关。true 时标定出 QA 质检图、
  测量落过程图（见 3.4）；false 为部署模式，只产出最终结果。

## 1. 功能 1：生成棋盘格标定板打印文件

头文件：`src/checkerboard/board_generator.h`

```cpp
bool cam::GenerateCheckerboardBoard(const std::string& iniPath, std::string& errMsg);
```

- 入参只有 iniPath，全部版面参数读 ini 的 `[checkerboard]` 段：
  `square_mm`（格边长 mm）、`cols`/`rows`（格数）、`border_mm`（留白）、
  `paper_w_mm`/`paper_h_mm`（纸张尺寸）、`out_dir`（产物目录）。
- 产物三件，落在 `[checkerboard] out_dir`：
  ▸ `checkerboard.pdf` 矢量打印文件（交打印店，100% 实际大小打印）
  ▸ `checkerboard_preview.png` 人工核对预览图（勿打印）
  ▸ `打印说明.txt` 随参数自动生成的打印要求说明
- 返回 false 时 errMsg 为中文原因（参数非法，如边距不足、格数奇偶相同；或写盘失败）。

## 2. 功能 2：棋盘格标定（生成标定文件）

头文件：`src/calibration/calibrator.h`

单张采图（常用）：

```cpp
bool cam::BuildCalibrationFile(const cv::Mat& boardImage,
                               const std::string& iniPath, std::string& errMsg);
```

多张采图取均值降噪（可选，固定机位连拍数张更稳）：

```cpp
bool cam::BuildCalibrationFileFromImages(const std::vector<cv::Mat>& boardImages,
                                         const std::string& iniPath, std::string& errMsg);
```

- 输入图像支持 8/24/32 通道（内部自动转灰度）。
- 参数读 ini 的 `[calibrate]` 段：`pattern_cols`/`pattern_rows`（内角点 = 格数-1）、
  `square_x_mm`/`square_y_mm`（实测格距，务必用卡尺量打印实物后回填）、
  `out_xml`（标定文件输出路径）、`target_mm_per_px`（正射刻度）、`qa_dir`（QA 图目录）。
- 产物：标定文件 `[calibrate] out_xml`（OpenCV XML：内参/畸变/外参/格距/正射刻度）；
  `[debug]` 打开时另有 4 张 QA 质检图落 `qa_dir`。
- 质量门禁：RMS > 0.3px 或校正验证 mean > 0.5px 时返回 false，但文件与 QA 图
  照常输出，errMsg 说明质量偏低。可据 QA 图检查印刷精度与采图条件后重标。
- `LoadCalibrationXml` 为内部加载函数，集成方一般不需要直接调（功能 3 的
  Rectifier 会自己加载）。

## 3. 功能 3：单张图像测量

头文件：`src/calibration/rectifier.h`、`src/measure/measure_pipeline.h`、
`src/measure_types.h`

调用模型：两个对象各初始化一次，之后每帧相机原图走一遍"先矫正、再测量"。

### 3.1 初始化（软件启动时各一次）

```cpp
cam::MeasurePipeline pipe;   // 测量流水线
cam::Rectifier rectifier;    // 几何矫正器（可选项，[rectify] enabled=true 时需要）

std::string err;
if (!pipe.Init(iniPath, err)) { /* err 为中文原因 */ }

// pipe.Init 内部已读取 ini 全量配置；用同一份配置判断矫正开关：
if (pipe.Config().rectify.enabled) {
    if (!rectifier.Load(pipe.Config().rectify.calib_xml,
                        pipe.Config().rectify.target_mm_per_px, err)) {
        /* 矫正器加载失败，中文原因 */
    }
}
```

- `MeasurePipeline::Init` 是重操作（背景建模 + ONNX 会话创建），只做一次。
- Init 内置模型预热，调用方无需做任何事，也没有配置开关：
  ▸ 触发时机：AI 分割会话创建成功后自动执行一次，Init 返回时预热已完成。
  ▸ 具体动作：用一张模型输入尺寸的合成纯灰图（灰度 128，尺寸跟随模型实际
    输入，如 384x384）跑一次完整推理，推理结果直接丢弃。计算图执行与图像
    内容无关，合成图足以触发全部首次开销。
  ▸ 覆盖的开销：cuDNN 卷积策略搜索、显存分配器扩张、CUDA 模块加载——首次
    推理慢的三大来源。不预热的话，现场第一帧测量会出现秒级卡顿。
  ▸ 如何确认生效：初始化日志出现"模型预热完成，首次推理耗时 xx ms"即预热
    成功；若预热抛异常，只打 Warn"模型预热失败（不影响后续推理）"，分割器
    仍为就绪状态。
  ▸ 补充说明：CPU 模式下预热同样执行，开销小、无害；纯灰图无产品检出属
    预期现象，InferCrop 返回 false 不算预热失败。
- 矫正开启时，Init 会自动用同一标定文件把背景模型同步正射校正，调用方无需处理。

### 3.2 逐帧测量（每收到一帧相机原图）

```cpp
cv::Mat raw = /* 相机原始图，8/24/32 通道均可 */;

// 第一步：几何矫正（开关关闭或矫正器未就绪时跳过，原图直进测量）
cv::Mat img = raw;
if (rectifier.IsReady()) {
    img = rectifier.Rectify(raw);   // 尺寸须与标定采图一致；失败返回空 Mat
    if (img.empty()) { /* 尺寸不符，见日志 */ }
}

// 第二步：纯测量（Measure 不关心图像是否矫正过）
cam::MeasureOutput out = pipe.Measure(img, "frame_tag");  // debugTag 可传 ""
if (out.code == cam::RetCode::OK) {
    // out.width.widthPx / out.height.heightPx / out.horizontalEdges 有效
} else {
    // out.message 为中文失败原因
}
```

- `Measure(image, debugTag)`：debugTag 仅在 `[debug] save_intermediate=true` 时生效，
  非空则向 `[paths] output_dir/debug/<debugTag>/` 落 6 张过程图（输入灰度/分割掩膜/
  增强图/旋转图/旋转掩膜/测量叠加图），便于部署现场排查；传 "" 则零中间文件。
- 像素换算：输出均为像素值。矫正开启时，毫米 = 像素 × `rectifier.MmPerPx()`
  （默认刻度 0.15 mm/px）。

### 3.3 输出数据结构（完整定义，与 `src/measure_types.h` 逐行一致）

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

### 3.4 测量相关 ini 段速查

- `[rectify]` 几何矫正开关与标定文件、`[paths]` 输入输出与背景缓存
- `[segmentation] method`：`ai`（ONNX，默认）或 `traditional`（传统背景差分）
- `[ai_seg]`：模型路径 / 阈值 / `device = auto|cuda|cpu`（auto 预检 CUDA 环境，
  缺 CUDA 12 运行时自动静默走 CPU）
- `[trad_seg]` / `[refine]` / `[rotate]` / `[measure]`：各算法参数，默认值与
  Python 版 `configs/default.yaml` 对齐

## 4. 集成注意事项

- 三个功能的菜单演示在 `main.cpp`，批量测量与 CSV 汇总也仅是演示代码；集成时
  按第 3 节的单帧流程对接即可。
- AI 分割模型缺失或加载失败时，流水线自动回退传统背景差分分割并记 Warn 日志，
  不会中断测量。
- 部署机跑 AI 分割需安装 CUDA 12.x + cuDNN 9.x 运行时（`device=auto` 会自动探测）；
  纯 CPU 运行无需任何额外依赖。
- 标定采图、背景建模、测量采图三者图像尺寸必须一致（同一相机同一分辨率）。

## 变更记录

- 2026-09-08：初版。覆盖制板 / 标定 / 单帧测量三大功能接口；测量部分按单帧调用
  编写（批量与 CSV 仅为 main.cpp 演示）；补充 [debug] 中间结果开关与 device=auto
  行为说明；功能 3 Init 内置模型预热说明（合成灰图首跑推理，消除首帧卡顿）；
  3.3 节改为输出数据结构的完整定义（Point2d/LineSegment/WidthResult/HeightResult/
  HorizontalEdge/MeasureOutput 全部列出），并补四线闭合矩形示意图与填充示例；
  3.1 节预热补充具体操作细节（触发时机/合成图内容/覆盖开销/日志观测点）。
