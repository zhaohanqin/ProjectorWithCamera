# 项目使用说明（基于当前架构）

## 一、项目结构与可执行程序

当前 CMake 生成以下可执行程序（目标名称）：
- projectorTest：功能测试集合（连接/投影/步进/自动生成条纹等）
- cameraTest：相机连接、参数保存与验证
- projectorWithCreame：投影仪与相机的完整协作示例（自动生成条纹 + 步进 + 采集保存）
- Projector_Crmera_test：单张图案投影与相机功能集合（全白投影拍照、实时预览调参等）

核心库/模块：
- module/projector_dlpc_api（投影仪底层控制，已在 CMake 中链接）
- OpenCV（图像处理/显示/保存）
- 海康 MVS SDK（`Includes/`、`Libraries/win64/`）

参数文件：
- camera_params.txt（位于构建输出目录，cameraTest/ProjectorWithCamera/Projector_Crmera_test 均读取/写入）

## 二、依赖与编译

1) 确保依赖路径正确（见根 `CMakeLists.txt`）：
- OpenCV_DIR 指向 OpenCV cmake 安装目录
- MVS_INCLUDES_DIR 与 MVS_LIBRARIES_DIR 指向海康 SDK 头文件与库目录

2) 使用 CMake 生成并编译（VS/MSVC）：
- 生成后，`cyusbserial.dll` 会被自动复制到可执行目录
- VS 调试路径中已附加 dlpc 的第三方 DLL 目录

## 三、camera_params.txt 格式

```ini
# 相机参数配置文件（键=值）
exposureTimeUs=10000.0       # 曝光时间（微秒）
exposureAutoMode=0           # 0=关闭，1=自动
gainValue=5.0                # 增益
gainAutoMode=0               # 0=关闭，1=自动
frameRate=10.0               # 帧率（fps）
triggerDelayUs=0             # 触发延时（微秒）
enableChunkData=0
printCurrentParams=1
```

cameraTest、ProjectorWithCamera、Projector_Crmera_test 都会读取/应用该文件；其中 Projector_Crmera_test 还能在实时预览中保存新参数。

## 四、各可执行的用途与使用

### 1) projectorTest
- 覆盖：连接、断开、project/pause/stop/step、LED 设置、自动生成条纹、按键步进等
- 默认注释/启用不同测试用例，按需在源文件中切换

### 2) cameraTest
- 连接相机、设置参数、保存到 `camera_params.txt`
- 运行后生成/更新参数文件，供其它程序使用

### 3) projectorWithCreame（完整协作示例）
- 自动生成 N 步相移条纹（垂直 N + 水平 N）
- 流程：投影仪加载 → 连续/步进控制 → 每步触发相机采集 → 保存图像到 `images/`
- 参数从 `camera_params.txt` 读取，可选择是否使用保存参数

### 4) Projector_Crmera_test（单张图案 + 实时调参）

提供菜单选择以下功能：
- 1 全白投影 + 单次拍照保存（保存到 `images/white_capture.png`）
- 2 投影单张全白 → 启动相机实时预览（可调参并保存到 `camera_params.txt`）
- 3 投影单张垂直条纹 → 启动相机实时预览（同上）

投影逻辑（与测试一致）：
- 非连续步进模式 `project(false)` + `step()`，随后 `pause()` 保持稳定，避免停留频闪

实时预览窗口：
- 尺寸：1080×720（缩放显示）
- 左上角叠加当前参数：曝光(us)/增益/帧率/触发延时
- 调参热键：
  - q/ESC：退出
  - s：保存当前参数到 `camera_params.txt`
  - +/-：曝光 ±5556us
  - g/G：增益 ±1
  - f/F：帧率 ±1 fps（下限 1）
  - t/T：触发延时 ±100us（下限 0）

## 五、常见问题与建议

- 分辨率匹配：加载到投影仪的图像必须与 DMD 分辨率一致（如 DLP4710 为 1920×1080）
- 灰度格式：建议使用 CV_8UC1 灰度图；若使用 1bit 模式需自行二值化并设置 `isOneBit_`
- 稳定显示：需要停留时使用步进+暂停，避免连续模式下可见频闪
- LED 亮度：用 `setLEDCurrent(r,g,b)` 控制；全白建议三色一致
- MVS 触发：实时预览使用连续采集（TriggerMode=Off）；单次拍照使用软件触发

## 六、快速上手

1) 运行 cameraTest 生成 `camera_params.txt`
2) 运行 Projector_Crmera_test → 菜单 2 或 3，开启实时预览并调参 → 按 s 保存
3) 运行 projectorWithCreame 进行完整步进协作与采集

本说明覆盖当前项目的实际可执行与功能入口，若需扩展其它投影图案或更复杂时序，可参考 `projectorTest` 与 `projectorWithCreame` 的实现方式进行复用与改造。
