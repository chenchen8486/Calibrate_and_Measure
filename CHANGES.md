# 版本变更清单

　　每交付一个版本，在既有条目之前追加一条变更记录（最新版本永远在最上面），列出相对上一版发生变化的全部文件。软件首次克隆整个工程后，后续升级只需按清单把新版同名文件复制覆盖到本地对应路径，并手动删除清单中标注删除的文件。

　　每条记录除文件清单外，另设“README 需要阅读的内容”一节，指明本版动了 README 哪些章节、软件照着要做什么，方便快速定位该读哪里，不必通读全文。

---

## v2.1.5（2026-09-22）

　　一句话：AI 分割漏检不再静默兜底出假数据，改报专用错误码 `AI_MISS`，三个对外接口的内部异常一律转为错误码返回，中间调试图默认关闭。

　　本次解决的问题：旧版遇到“板上有物体但模型认不出目标产品”时，会退回传统粗掩膜，把杂物当产品，输出一组看起来正常的假宽高，调用方无法分辨。另外接口内部异常会穿透到宿主程序，表现为无返回、卡死或闪退，且拿不到任何错误码。这两处本版都改掉了。

### 一、要替换的源码文件（7 个，替换后须重新编译）

| 文件 | 变化说明 |
|---|---|
| `src/measure_types.h` | 新增返回码 `AI_MISS = 5` |
| `src/measure/segment_ai.h` | Segment 新增 failCode 输出参数，删除漏检退回粗掩膜约定 |
| `src/measure/segment_ai.cpp` | AI 漏检返回 AI_MISS，ONNX 推理异常返回 INTERNAL，不再兜底出数 |
| `src/measure/measure_pipeline.h` | Measure 返回码说明补 AI_MISS / NO_BACKGROUND 与异常兜底约定 |
| `src/measure/measure_pipeline.cpp` | AI 分割失败按原因码透传，补 `catch(...)` 未知异常兜底 |
| `src/cam_api.h` | 错误模型注释更新，版本号 2.1.5 |
| `src/cam_api.cpp` | Init / Measure / SetBackground 三个入口全面 try/catch，版本号 2.1.5 |

　　另需更新数据文件 `temp/background_model.bmp`（背景缓存，已同步为当前场景空板图，克隆仓库即带最新版。现场换场景后按 README 6.1 节用 SetBackground 重学）。

### 二、配置参数变更（config.ini 手工合并，勿整文件覆盖）

| 参数 | 旧值 | 新值 | 说明 |
|---|---|---|---|
| `[debug] save_intermediate` | `true` | `false` | 部署模式，不再落分割掩膜、增强图、旋转图、测量叠加图等中间调试图，只输出最终结果（标定 XML、测量 CSV）。排查问题时临时改回 `true` 即可 |

### 三、README 需要阅读的内容

| 章节 | 本版相关内容 | 软件要做什么 |
|---|---|---|
| 第 6 节 返回码表 | 新增 `AI_MISS = 5` | 在 `code` 分支里加上这一档 |
| 第 7 节 集成注意事项 | 新增三条：AI_MISS 的处理姿势、`NO_PRODUCT` 与 `AI_MISS` 的区别（传统链路不产生 AI_MISS）、无返回/卡死时照 DLL 版本排查 | 按该节把返回码分支补齐 |
| 第 9 节 依赖与分发清单 | 运行时 dll 必须放 exe 同目录的原因说明 | 核对部署目录下 `onnxruntime.dll` 等文件的版本与位置 |

### 四、文件增删

　　新增：本版无。
　　删除：本版无。

### 五、替换后的操作步骤

1. 用新版覆盖第一节表中的 7 个源码文件。
2. 按第二节手工改 `config.ini` 的那一项（只改这一项，勿整文件覆盖）。
3. Release x64 重新编译。
4. 核对 exe 同目录的 `onnxruntime.dll`、`opencv_world480.dll` 是否为随包版本。System32 或 PATH 里的旧版同名 dll 会被 Windows 优先加载，导致初始化崩溃且无任何错误码返回（本机已实测复现）。
5. 若软件要处理 AI_MISS：收到后按“疑似放错物料或模型失效”提示人工复核，本帧不要触发任何硬件调整。

---

## v2.1.3（2026-09-22）

　　变更记录：Measure 新增可选输出旋转校正后的测量基准图（第 3 个参数 basisImage）。传指针即拿到测量实际使用的那张图，灰度 8UC1，不画任何标注。正射矫正是否参与由 [rectify] enabled 决定，与原来一致。不传参数时行为与旧版完全一致。

### 要替换的文件（5 个，含源码，替换后须重新编译）

| 文件 | 变化说明 |
|---|---|
| `src/cam_api.h` | Measure 新增 basisImage 可选参数，版本号 2.1.3 |
| `src/cam_api.cpp` | Measure 透传 basisImage（失败时输出置空），版本号 2.1.3 |
| `src/measure/measure_pipeline.h` | Measure 新增 basisImage 参数声明 |
| `src/measure/measure_pipeline.cpp` | 旋转校正后填充基准图（未画标注，正射参与与否随输入） |
| `README.md` | 6.2 节基准图输出示例与说明，接口总览表同步 |

### 要新增的文件

　　本版无。

### 要删除的文件

　　本版无。

### 配置参数变更（config.ini 手工合并，勿整文件覆盖）

　　无变更，config.ini 无需替换。

### 替换后要做的事

　　重新编译（Release x64）。软件代码不改也能编译通过（新参数有默认值），需要基准图时给 Measure 传第 3 个参数即可。

---

## v2.1.2（2026-09-20）

　　变更记录：背景建模交付化（缓存优先、新增 SetBackground 现场学习接口、生产与 demo 分家，Init 不再用 input_dir 现建背景），功能 2 标定质检修复（质检图与校正验证首次真正产出）。

### 要替换的文件（12 个，含源码，替换后须重新编译）

| 文件 | 变化说明 |
|---|---|
| `src/measure/measure_pipeline.h` | 新增 SetBackground 声明 |
| `src/measure/measure_pipeline.cpp` | 背景加载生产/demo 分家，新增 SetBackground |
| `src/cam_api.h` | SetBackground 接口，版本号 2.1.2 |
| `src/cam_api.cpp` | SetBackground 接口，版本号 2.1.2 |
| `src/measure_types.h` | 新增返回码 NO_BACKGROUND，背景未就绪时 Measure 返回它 |
| `src/measure/background.h` | 背景缓存加载函数 |
| `src/measure/background.cpp` | 背景缓存加载函数 |
| `main.cpp` | demo 显式中位数建模 |
| `src/calibration/calibrator.cpp` | 功能 2 质检修复 |
| `README.md` | 接口说明更新，6.1 节新增 SetBackground 调用说明 |
| `.gitignore` | temp/ 仅放行 background_model.bmp 入库 |
| `CHANGES.md` | 本文件，变更清单更新 |

### 要新增的文件（1 个）

| 文件 | 说明 |
|---|---|
| `temp/background_model.bmp` | 背景缓存，到新版工程 temp/ 下取 |

### 要删除的文件

　　本版无。

### 配置参数变更（config.ini 手工合并，勿整文件覆盖）

　　无有效键变更，仅 `[paths]` 注释更新（说明 input_dir 为 demo 专用），可不替换。

### 替换后要做的事

　　重新编译（Release x64）。背景缓存沿用即可，软件集成按“启动 → 抓一帧 → SetBackground”做开班刷新，每次启动都执行，不要包在未就绪判断里。

---

## v2.1.1（2026-09-20）

　　变更记录：正射刻度统一为 0.136 mm/px，与现场软件口径对齐。无算法行为变化。

### 要替换的文件（6 个，含源码，替换后须重新编译）

| 文件 | 变化说明 |
|---|---|
| `config.ini` | 见下方“配置参数变更” |
| `src/common/ini_config.h` | 默认值同步 0.136 |
| `src/cam_api.cpp` | 版本号 2.1.1 |
| `src/cam_api.h` | 版本号 2.1.1 |
| `README.md` | 刻度描述同步，加本清单导引 |
| `.gitignore` | 忽略编译散件与本地备份目录 |

### 要新增的文件（1 个）

| 文件 | 说明 |
|---|---|
| `CHANGES.md` | 本文件，新版工程根目录下取 |

### 要删除的文件（1 个）

| 文件 | 说明 |
|---|---|
| `assets/test_data/input/5.jpg` | 测试数据移出仓库 |

### 配置参数变更（config.ini 手工合并，勿整文件覆盖）

| 段 | 键 | 旧值 | 新值 | 说明 |
|---|---|---|---|---|
| `[calibrate]` | `target_mm_per_px` | 0.15 | 0.136 | 正射输出刻度，与现场软件写死的像素当量对齐 |

### 替换后要做的事

　　重新编译（Release x64），并用功能 2 重新生成标定 XML。毫米换算为 像素 × 0.136。
