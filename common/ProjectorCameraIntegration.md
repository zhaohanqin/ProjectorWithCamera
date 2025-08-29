# 投影仪-相机协作与测试说明（软触发）

本文面向当前代码实现，详细介绍以下三个文件：

- `common/ProjectorWithCamera.cpp`：投影仪与相机软触发协作的完整样例（“投影一张 → 相机拍一张”）。
- `common/ProjectorTest.cpp`：投影仪功能的完整测试套件（连接/投影/图案加载/LED 等）。
- `common/CameraTest.cpp`：相机连接与抓拍测试（可选序列号、回调保存 I1..I8）。

## 1) ProjectorWithCamera.cpp（协作样例）

### 作用

- 以“软触发步进”的方式让投影仪与相机协作：每步投影显示一张条纹图后，软件触发相机拍摄一帧并保存。
- 自动生成 2N 张相移条纹（垂直 N + 水平 N），装载至投影仪后按序步进投影。

### 入口函数

```cpp
bool runProjectorCameraCooperation(
    const std::string& projectorModel,
    int deviceWidth,
    int deviceHeight,
    int steps,
    int frequency,
    int intensity,
    int offset,
    double noiseStd,
    const std::string& cameraSerial,
    const std::string& outputDir,
    double exposureTimeUs,
    double gainValue,
    double acquisitionFps,
    double triggerDelayUs
)
```

### 参数说明

- projectorModel：投影仪型号（默认 DLP4710）。
- deviceWidth/deviceHeight：投影分辨率，须与设备一致（如 1920x1080）。
- steps：相移步数 N（总投影帧数为 2N）。
- frequency/intensity/offset/noiseStd：条纹生成参数（正弦相移条纹）。
- cameraSerial：相机序列号；传 "NULL" 或空字符串表示自动选择第一台。
- outputDir：图片输出目录；空字符串表示当前工作目录。
- exposureTimeUs/gainValue/acquisitionFps/triggerDelayUs：
  - 均为相机可选参数；传负数表示未指定，保持相机自动/默认；传非负数则关闭自动并按值设置。

### 参数影响（成像与时序）

- 曝光时间 ExposureTime(us)：积分时间；增大→更亮/更易拖影/步进周期需更长；减小→更暗/更清晰/更快步进。
- 增益 Gain：提升亮度同时放大噪声；优先用曝光满足亮度，增益仅作补偿。
- 采集帧率 AcquisitionFrameRate：在触发模式下常作为上限/定时参考，需与“曝光+读出+传输”匹配，否则易丢帧或阻塞。
- 触发延时 TriggerDelay(us)：从触发到开始曝光的延时；用于对齐 `projector->step()` 后的稳定窗口，避免采到过渡帧。

### 主流程（概述）

1. 连接投影仪并获取实例（`ProjectorFactory`）。
2. 构造 2N 张正弦相移条纹图并封装为两个 `PatternOrderSet`（垂直 N、水平 N）。
3. `populatePatternTableData(...)` 写入设备；`project(true)` 开始投影。
4. 初始化相机：枚举设备→按序列号选择→创建句柄→打开；
   - 设置像素格式 Mono8；
   - 对四个可选参数：仅当传入非负数时才关闭自动并夹紧范围后设置；
   - 注册图像回调（保存 I1..I(2N).png）；`StartGrabbing()` 开始采集。
5. 循环 2N 次：`step()` 投影下一帧→等待稳定（典型几十~数百毫秒）→ `TriggerSoftwareExecute()` 软触发相机。
6. 完成后停止投影并清理相机（Stop/Close/DestroyHandle），断开投影仪。

### 示例调用（main）

- 相机四个参数在示例中默认 -1（未指定，保持自动）。修改为非负数即生效。

## 2) ProjectorTest.cpp（投影仪测试套件）

### 作用

- 面向投影仪的功能验证：
  - 初始化与信息获取
  - 连接/断开、连接状态检查
  - 单次/连续投影
  - 步进投影
  - 图案数据加载（内部闪存）
  - LED 电流读写

### 主要测试项（函数）

- `testProjectorInit()`：工厂能否创建实例。
- `testProjectorGetInfo()`：查询设备信息。
- `testProjectorConnect()/testProjectorDisconnect()/testProjectorIsConnect()`：连接相关。
- `testProjectorOnceProject()/testProjectorContinueProject()/testProjectorStop()`：投影控制。
- `testProjectorPopulatePatternTableData()`：加载用户图案至设备；随后 `project(true)` 验证。
- `testProjectorStep()/testProjectorStepWithGeneratedFringes()`：步进投影与自动生成条纹流程。
- `testProjectorGetSetLEDCurrent()`：LED 亮度读写。

### 运行

- 入口 `main()` 调用 `runAllTests()`，并统计通过/失败数。

## 3) CameraTest.cpp（相机测试）

### 作用

- 验证相机链路与软触发采集：
  - 枚举相机并按序列号选择（传 "NULL" 表示第一台）。
  - 打开设备后配置像素格式 Mono8。
  - 当用户在 main 中提供以下参数为非负数时，关闭自动并按值设置：
    - 曝光时间 ExposureTime(us)、增益 Gain、采集帧率 AcquisitionFrameRate、触发延时 TriggerDelay(us)。
  - 注册图像回调保存 I1..IN.png（默认 N=8）。

### 入口测试函数

```cpp
static bool runCameraTest(
    const std::string& cameraSerial,
    const std::string& outputDir,
    int framesToCapture,
    double exposureTimeUs,
    double gainValue,
    double acquisitionFps,
    double triggerDelayUs)
```

### main 参数示例

- 默认：`exposureUs/gain/fps/trigDelayUs = -1` 表示未指定（保持自动/默认）。
- 修改为非负数即生效，并在控制台打印设置值与可用范围。

### 采集流程

1. 枚举并选择设备（USB/GigE 序列号均支持）。
2. 创建句柄并打开设备，设置 Mono8、TriggerMode=On、TriggerSource=Software。
3. 依据参数决定是否关闭自动并设置曝光/增益/帧率/触发延时。
4. 注册回调并开始抓取；循环软触发 N 次，保存 I1..IN.png。
5. 停止抓取与关闭设备。

## 4) 条纹与时序建议

- 条纹生成参数（`frequency/intensity/offset/steps`）需与投影仪分辨率与算法期望一致；
- 协作时序：建议“先投影 step → 等稳定 → 触发相机”，稳定时间可按 `preExposureTime_/postExposureTime_` 结合实测确定；
- 曝光时间应不超过投影稳定窗口，避免过渡帧；
- 帧率设置需与“曝光+读出+传输”匹配，防止丢帧。

## 5) 编译与运行

- CMake 已配置三个可执行文件：
  - `projectorTest`（投影仪测试）
  - `projectorWithCreame`（投影-相机协作样例）
  - `cameraTest`（相机测试）
- 已链接 OpenCV 与 MVS SDK；并在 VS 调试环境中添加了所需 DLL 的路径。
- 若 IDE 提示找不到 OpenCV 头文件，请确认 `OpenCV_DIR` 指向含 `OpenCVConfig.cmake` 的目录，并重新生成构建。

## 6) 常见问题

- 相机未发现：检查驱动、权限、线缆与供电；确认 MVS 客户端能识别设备。
- 抓拍黑图/过曝：调整 Exposure/Gain；确认投影稳定时间与 TriggerDelay 是否匹配。
- 丢帧：缩短曝光、降低 fps、提升带宽或分辨率降级；确保保存目录在 SSD 且无写入权限问题。
