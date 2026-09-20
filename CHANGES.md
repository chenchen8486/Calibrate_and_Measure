# 版本变更清单

　　每交付一个版本，在既有条目之前追加一条变更记录（最新版本永远在最上面），列出相对上一版发生变化的全部文件。软件首次克隆整个工程后，后续升级只需按清单把新版同名文件复制覆盖到本地对应路径，并手动删除清单中标注删除的文件。

## v2.1.2（2026-09-20）

　　变了啥：背景建模交付化（缓存优先、新增 SetBackground 现场学习接口、生产与 demo 分家，Init 不再用 input_dir 现建背景），功能 2 标定质检修复（质检图与校正验证首次真正产出）。

　　要替换的文件（新版覆盖本地同名文件，共 12 个，含源码，替换后须重新编译）：

- src/measure/measure_pipeline.h、src/measure/measure_pipeline.cpp（背景加载生产/demo 分家，新增 SetBackground）
- src/cam_api.h、src/cam_api.cpp（SetBackground 接口，版本号 2.1.2）
- src/measure_types.h（新增返回码 NO_BACKGROUND，背景未就绪时 Measure 返回它）
- src/measure/background.h、src/measure/background.cpp（背景缓存加载函数）
- main.cpp（demo 显式中位数建模）
- src/calibration/calibrator.cpp（功能 2 质检修复）
- README.md（接口说明更新，6.1 节按主题分块重排）
- .gitignore（temp/ 仅放行 background_model.bmp 入库）
- CHANGES.md（本文件，变更清单更新）

　　要新增的文件（拷入本地对应位置，共 1 个）：

- temp/background_model.bmp（背景缓存，到新版工程 temp/ 下取）

　　要删除的文件（本地手动删除）：

- 无

　　配置参数变更（config.ini 手工合并，勿整文件覆盖）：

- 无有效键变更，仅 [paths] 注释更新（说明 input_dir 为 demo 专用），可不替换。

　　替换后要做的事：重新编译（Release x64）。背景缓存沿用即可，软件集成按“启动 → 抓一帧 → SetBackground”做开班刷新，每次启动都执行，不要包在未就绪判断里。

## v2.1.1（2026-09-20）

　　变了啥：正射刻度统一为 0.136 mm/px，与现场软件口径对齐。无算法行为变化。

　　要替换的文件（新版覆盖本地同名文件，共 6 个，含源码，替换后须重新编译）：

- config.ini（见下方参数变更）
- src/common/ini_config.h（默认值同步 0.136）
- src/cam_api.cpp、src/cam_api.h（版本号 2.1.1）
- README.md（刻度描述同步，加本清单导引）
- .gitignore（忽略编译散件与本地备份目录）

　　要新增的文件（拷入本地对应位置，共 1 个）：

- CHANGES.md（本文件，新版工程根目录下取）

　　要删除的文件（本地手动删除，共 1 个）：

- assets/test_data/input/5.jpg（测试数据移出仓库）

　　配置参数变更（config.ini 手工合并，勿整文件覆盖）：

- [calibrate] target_mm_per_px：由 0.15 改为 0.136（正射输出刻度，与现场软件写死的像素当量对齐）

　　替换后要做的事：重新编译（Release x64），并用功能 2 重新生成标定 XML。毫米换算为 像素 × 0.136。
