# Calibrate_and_Measure

印刷品尺寸测量 C++ 工程：棋盘格标定 + 图像测量（宽 / 高 / 上半部分水平边长）。
算法由 Python 版 `D:/project/test_edge_detect` 等价移植，接口对齐其 `INTERFACE.md` v1.4。

## 三大功能

| 功能 | 入口 | 说明 |
|---|---|---|
| 1 制板 | `cam::GenerateCheckerboardBoard(iniPath, errMsg)` | 生成棋盘格 PDF + 预览图 + 打印说明.txt |
| 2 标定 | `cam::BuildCalibrationFile(image, iniPath, errMsg)`（单图）<br>`cam::BuildCalibrationFileFromImages(images, iniPath, errMsg)`（多图取均值） | SB 角点 + calibrateCamera → 标定 XML + QA 质检图 |
| 3 测量 | `cam::Rectifier`（几何矫正）+ `cam::MeasurePipeline`（纯测量） | 原图先 `Rectifier::Rectify` 矫正，再 `MeasurePipeline::Measure` 测量 |

对外数据类型见 `src/measure_types.h`（`MeasureOutput`：宽度 / 高度 / 水平边列表 / 返回码）。
集成调用约定（含单帧测量流程与示例代码）见 `INTERFACE.md`；
三大功能的流程图见 `docs/design/2026-09-09-三大功能流程图.md`（附 PNG 渲染图）。

## 目录结构

```
Calibrate_and_Measure/
├── Calibrate_and_Measure.sln / .vcxproj   # VS2019 (v142) 工程
├── opencv_onnx_paths.props                # ★ 第三方库路径（唯一需要手动改的文件）
├── config.ini                             # ★ 全工程配置（按功能分段，中文注释）
├── main.cpp                               # 菜单驱动入口（仿 Python run.py）
├── src/
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

## 环境要求

- Visual Studio 2019（v142），C++17
- OpenCV 4.8.0（`findChessboardCornersSB` 需要 ≥4.5）
  → 本机已配好：`D:\3_package\dev_config\opencv\build`（vc16 库，v142 兼容）
- ONNX Runtime GPU 版 1.20.1（AI 分割用；模型缺失时自动回退传统分割）
  → 本机已配好：`D:\3_package\dev_config\onnxruntime-gpu`
- CUDA 12.x + cuDNN 9.x：本机仅编译链接（`cudart.lib`），部署机运行 AI 推理时必须安装
  （ONNX Runtime CUDA EP 运行时依赖 `cudart64_12x.dll` 与 cuDNN 9 的 dll）。
  `[ai_seg] device=auto`（默认）会先试装 `onnxruntime_providers_cuda.dll` 预检环境，
  缺 CUDA 运行时则静默走 CPU，不会再刷 ORT 英文报错；`cuda` 为强制 GPU。

首次使用：打开 `opencv_onnx_paths.props`，把 `OPENCV_DIR` 和 `ONNXRUNTIME_DIR` 改成本机实际路径即可，其余不用动。

## 使用流程

1. 运行 `Calibrate_and_Measure.exe` 进入菜单：
   - `1` 生成标定板 → 把 `assets/checkerboard/打印说明.txt` 和 PDF 交打印店（100% 实际大小打印）
   - `2` 打印板铺在吸风展平板上采图放入 `assets/test_data/calibration/`，执行标定 → 生成 `assets/calibration/checkerboard_calib.xml` + QA 图（`temp/calibration_qa/`，受 `[debug]` 开关控制）
   - 也可以把单张采图直接拖到 exe 上快捷标定
   - 用卡尺实测打印格距，回填 `config.ini` 的 `[calibrate] square_x_mm / square_y_mm`
2. 标定达标后把 `config.ini` 的 `[rectify] enabled` 改为 `true`
3. 待测产品图放入 `assets/test_data/input/`，菜单选 `3` 批量测量：
   - 控制台逐张打印结果；汇总写 `assets/test_data/output/measure_results.csv`
     （含宽高四线端点坐标），每条候选水平边的坐标与长度写 `measure_edges.csv`
   - `[debug] save_intermediate=true` 时每张图的过程图（掩膜/增强/旋转/测量叠加）
     落在 `assets/test_data/output/debug/<图像名>/` 下，部署时可关闭

## 配置说明

全部参数集中在 `config.ini`，按功能分段：

- `[checkerboard]` 功能 1 制板：格距 / 格数 / 纸张 / 白边（默认 15mm 格、35×28、555×450mm）
- `[calibrate]` 功能 2 标定：采图目录 / 内角点 / 实测格距 / 输出 XML / QA 目录
- `[rectify]` 功能 3-0 几何矫正开关：标定文件 / 正射刻度（默认 0.15 mm/px）
- `[paths]` 输入输出路径：待测图目录 / 结果目录 / 背景模型缓存
- `[segmentation]` 分割方式：`ai`（默认，ONNX）或 `traditional`
- `[ai_seg]` AI 分割：ONNX 模型路径 / 阈值 / 推理设备（`auto` 自动探测 CUDA、`cuda`、`cpu`）
- `[trad_seg]` 传统分割：双边滤波 / gamma / 差分阈值区间 / 形态学 / 最小面积
- `[refine]` 亚像素边缘精修、`[rotate]` 角度校正、`[measure]` 测量判定参数
- `[debug]` 中间结果落盘开关：`save_intermediate=false` 时只产出最终结果
  （标定 XML、测量 CSV），不落 QA 质检图与测量过程图；背景模型缓存是功能性缓存，不受此开关控制

默认值与 Python 版 `configs/default.yaml` 对齐。

## 已知说明

- 标定质量门禁：RMS ≤ 0.3px 且校正验证 mean ≤ 0.5px，越限时产物保留但返回失败并给中文原因。
- AI 分割的 ONNX 模型由 Python 侧导出（待补充）；导出前请把 `[segmentation] method` 设为 `traditional`，或保持 `ai`（模型缺失会自动回退传统并记 Warn 日志）。
- RANSAC 随机源为 `cv::RNG(42)`，与 Python numpy PCG64(42) 统计语义等价但样本不同，与 Python 对拍可能存在亚像素级差异。
- 编译时 main.cpp 可能出现 C4819/C4477 警告（MSVC 对 UTF-8 中文字面量 printf 格式检查的误报），不影响功能。

## 变更记录

- 2026-09-09：新增 `docs/design/2026-09-09-三大功能流程图.md`，制板 / 标定 / 单帧测量三张 Mermaid 流程图（与代码逐模块核对），附 mmdc 渲染的 PNG 供无 Mermaid 环境直接查看。
- 2026-09-08（4）：AI 分割会话创建后内置模型预热（合成灰图跑通一次完整推理，消除现场首帧秒级卡顿；不加配置开关，默认即做，预热失败仅记 Warn 不影响就绪）。
- 2026-09-08（3）：新增 `INTERFACE.md` 集成调用接口说明（制板 / 标定 / 单帧测量，含 MeasureOutput 字段表与示例代码）。
- 2026-09-08（2）：`test_data/` 并入 `assets/`（采图/待测图/结果统一到 `assets/test_data/`）；`[ai_seg] device` 新增 `auto`（默认）：预检 CUDA 运行时可用才注册 CUDA EP，缺运行时静默走 CPU，消除 error 126 噪音；新增 `[debug] save_intermediate` 总开关，统一接管标定 QA 质检图与测量过程图（`output/debug/<图像名>/`），部署置 false 即零中间文件；测量 CSV 拆为 `measure_results.csv`（汇总，含宽高四线端点坐标）+ `measure_edges.csv`（每条水平边的端点坐标/长度/判定）。
- 2026-09-08：初版建立。自 FastBilateralNPP 工程改名重建；完成制板 / 标定 / 矫正 / 测量（传统链 + AI 接口）全模块移植；13 个编译单元离线语法核查通过（OpenCV 3.0 头文件 + shim 环境）。OpenCV 4.8.0（D:\3_package\dev_config\opencv\build）与 ONNX Runtime GPU 1.20.1（D:\3_package\dev_config\onnxruntime-gpu）路径已配入 props；工程不直接链接 CUDA（GPU 推理由 ORT CUDA EP 承担，部署机需 CUDA 12 + cuDNN 9 运行时）。待实机 VS 编译与 Python 对拍验证。
