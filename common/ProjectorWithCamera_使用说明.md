# ProjectorWithCamera 项目完整使用说明

## 一、项目概述

本项目是一个基于DLP投影仪和工业相机的结构光三维重建系统，支持投影仪与相机的精确同步控制，用于3D扫描、表面检测、相位测量等应用。

### 核心功能
- **投影仪控制**：支持DLP4710/DLP3010等型号，提供连续投影、步进投影、LED亮度控制等功能
- **相机控制**：支持海康MVS工业相机，提供参数调节、软触发采集、实时预览等功能
- **同步协作**：投影仪步进投影与相机软触发采集的精确同步
- **图案生成**：自动生成N步相移条纹图案（垂直+水平）
- **参数管理**：相机参数的保存、加载和实时调节

## 二、项目结构

```
common/
├── ProjectorWithCamera.cpp     # 主程序：投影仪与相机完整协作
├── ProjectorTest.cpp          # 投影仪功能测试集合
├── CameraTest.cpp            # 相机功能测试和参数配置
├── Projector_Crmera_test.cpp # 单图案投影+相机实时预览调参
├── projector.h               # 投影仪接口定义
├── projectorFactory.h        # 投影仪工厂类
├── typeDef.h                 # 类型定义
└── ProjectorWithCamera_使用说明.md  # 本文档

module/
└── projector_dlpc_api/       # 投影仪底层控制模块
    ├── include/              # 头文件
    ├── src/                  # 源文件
    └── third_party/          # 第三方依赖

Includes/                     # 海康MVS相机SDK头文件
Libraries/                    # 海康MVS相机SDK库文件
images_Projector/            # 测试图案文件
```

## 三、编译与依赖

### 依赖环境
- **CMake** 3.10+
- **OpenCV** 4.x（图像处理和显示）
- **海康MVS SDK**（工业相机控制）
- **Visual Studio 2019+**（Windows编译）

### 编译步骤
1. 配置CMake变量：
   ```cmake
   OpenCV_DIR=C:/opencv/build
   MVS_INCLUDES_DIR=./Includes
   MVS_LIBRARIES_DIR=./Libraries/win64
   ```

2. 生成项目：
   ```bash
   mkdir build && cd build
   cmake .. -G "Visual Studio 16 2019" -A x64
   ```

3. 编译：
   ```bash
   cmake --build . --config Release
   ```

### 生成的可执行程序
- `projectorTest.exe` - 投影仪功能测试
- `cameraTest.exe` - 相机功能测试
- `projectorWithCreame.exe` - 投影仪与相机协作主程序
- `Projector_Crmera_test.exe` - 单图案投影与实时调参

## 四、核心文件详细说明

### 4.1 ProjectorWithCamera.cpp - 主协作程序

#### 功能概述
实现投影仪与相机的完整同步协作流程，自动生成N步相移条纹并进行精确的步进投影+软触发采集。

#### 主要功能
- **自动条纹生成**：生成垂直和水平相移条纹图案
- **同步投影采集**：投影仪步进与相机软触发的精确同步
- **参数管理**：从`camera_params.txt`加载相机参数
- **图像保存**：自动保存采集的图像到指定目录

#### 关键参数
```cpp
// 投影仪参数
const std::string projectorModel = "DLP4710";  // 投影仪型号
const int deviceWidth = 1920;                  // 投影分辨率宽度
const int deviceHeight = 1080;                 // 投影分辨率高度

// 条纹生成参数
const int steps = 4;                           // 相移步数（生成2N张图像）
const int frequency = 15;                      // 条纹频率（周期数）
const int intensity = 100;                     // 条纹强度（振幅0-127）
const int offset = 128;                        // 亮度偏移（平均灰度0-255）
const double noiseStd = 0.0;                   // 噪声标准差

// 投影仪时序参数（微秒）
patternSets[0].exposureTime_ = 4000;           // 曝光时间
patternSets[0].preExposureTime_ = 3000;        // 预曝光时间
patternSets[0].postExposureTime_ = 3000;       // 后曝光时间

// LED参数
projector->setLEDCurrent(0.9, 0.9, 0.9);      // RGB LED电流（0.0-1.0）
```

#### 图像命名规则
- **垂直条纹**：`I1.png` ~ `IN.png`（N为步数）
- **水平条纹**：`I(N+1).png` ~ `I(2N).png`
- 例如4步相移：垂直I1-I4，水平I5-I8

#### 运行方式
```bash
./projectorWithCreame.exe
```

程序会自动：
1. 连接投影仪和相机
2. 加载相机参数
3. 生成条纹图案
4. 执行垂直条纹投影+采集
5. 执行水平条纹投影+采集
6. 保存所有图像到`images/`目录

### 4.2 ProjectorTest.cpp - 投影仪测试程序

#### 功能概述
提供投影仪的全面功能测试，包括基础控制、图案加载、步进投影、按键触发等功能。

#### 主要测试功能
1. **基础功能测试**
   - 投影仪初始化和连接
   - 投影控制（开始/停止/暂停/恢复）
   - LED电流控制和获取

2. **图案投影测试**
   - 自定义图案加载和投影
   - 自动生成条纹图案投影
   - 步进投影控制

3. **按键触发投影**
   - 垂直条纹按键步进投影
   - 水平条纹按键步进投影
   - 全白图案连续投影

4. **全白图案投影**
   - 单色全白图案投影
   - 按ESC键退出控制

#### 关键参数配置
```cpp
// 投影仪型号选择
const std::string testProjector4710 = "DLP4710";
const std::string testProjector3010 = "DLP3010";

// 设备分辨率
const int deviceWidth = 1920;   // DLP4710分辨率
const int deviceHeight = 1080;

// 条纹生成参数
const int steps = 4;            // 相移步数
const int frequency = 15;       // 条纹频率
const int intensity = 100;      // 条纹强度
const int offset = 128;         // 亮度偏移

// LED电流设置（影响亮度）
projector->setLEDCurrent(0.9, 0.9, 0.9);    // 90%亮度
projector->setLEDCurrent(0.95, 0.95, 0.95); // 95%亮度（更亮）
```

#### 按键控制说明
- **空格键**：进行下一步投影
- **ESC键**：退出当前测试
- **其他键**：显示当前状态

#### 运行方式
```bash
./projectorTest.exe
```

在源码中可通过注释/取消注释来选择运行的测试：
```cpp
void runAllTests() {
    // 取消注释需要运行的测试
    testProjectorStepWithGeneratedVerticalFringesKeyTrigger();
    testProjectorStepWithGeneratedHorizontalFringesKeyTrigger();
    testProjectorWhitePatternProjection();
    testProjectorGetSetLEDCurrent();
}
```

### 4.3 CameraTest.cpp - 相机测试程序

#### 功能概述
提供工业相机的全面测试和参数配置功能，生成和管理`camera_params.txt`配置文件。

#### 主要功能
1. **相机基础测试**
   - 相机枚举和连接
   - 参数范围获取
   - 软触发兼容性测试

2. **参数配置测试**
   - 不同曝光时间测试
   - 不同增益值测试
   - 参数保存和加载

3. **图像采集测试**
   - 单帧采集和保存
   - 多帧连续采集
   - 端到端完整测试

#### 相机参数详解
```cpp
struct CameraParams {
    // 曝光参数
    float exposureTimeUs = -1.0f;        // 曝光时间（微秒）
                                         // 范围：通常20-100000μs
                                         // 影响：时间越长图像越亮，但运动模糊增加
    bool exposureAutoMode = false;       // 自动曝光开关
    
    // 增益参数  
    float gainValue = -1.0f;             // 增益值（dB）
                                         // 范围：通常0-30dB
                                         // 影响：增益越大图像越亮，但噪声增加
    bool gainAutoMode = false;           // 自动增益开关
    
    // 帧率参数
    float frameRate = -1.0f;             // 帧率（fps）
                                         // 范围：取决于曝光时间和相机性能
                                         // 影响：帧率越高采集速度越快
    
    // 触发参数
    int triggerDelayUs = 0;              // 触发延时（微秒）
                                         // 用于投影仪与相机的时序同步
};
```

#### 参数调节建议
- **室内环境**：曝光20000μs，增益8.0
- **明亮环境**：曝光10000μs，增益4.0  
- **暗光环境**：曝光50000μs，增益12.0
- **高速采集**：曝光5000μs，增益15.0

#### 运行方式
```bash
./cameraTest.exe
```

程序会自动运行所有测试并生成`camera_params.txt`文件。

### 4.4 Projector_Crmera_test.cpp - 实时调参程序

#### 功能概述
提供单图案投影与相机实时预览功能，支持实时参数调节和保存。

#### 主要功能菜单
```
请选择功能:
1) 全白投影+单次拍照保存
2) 投影单张(全白)并启动相机实时预览可调参  
3) 投影单张(垂直条纹)并启动相机实时预览可调参
4) 投影单张(水平条纹)并启动相机实时预览可调参
```

#### 实时调参界面
- **显示窗口**：1080×720相机预览窗口
- **参数显示**：实时显示当前曝光、增益、帧率等参数
- **图像统计**：显示平均亮度、标准差、最值等信息
- **直方图**：300×80像素的亮度分布直方图

#### 按键控制说明
```cpp
// 参数调节按键
'8' - 增加曝光时间 +1000μs
'2' - 减少曝光时间 -1000μs  
'6' - 增加增益 +0.5dB
'4' - 减少增益 -0.5dB

// 功能按键
's' - 保存当前参数到camera_params.txt
'q'/'ESC' - 退出程序
```

#### 图案类型配置
```cpp
// 全白图案
cv::Mat pattern = cv::Mat(H, W, CV_8UC1, cv::Scalar(255));

// 垂直条纹图案  
pattern = generateStripeImage(W, H, true, frequency, 100, 128);

// 水平条纹图案
pattern = generateStripeImage(W, H, false, frequency, 100, 128);
```

#### 条纹频率设置
- **低频率（5-10）**：适合粗测量，条纹宽度大
- **中频率（15-25）**：适合一般应用，平衡精度和稳定性
- **高频率（30-50）**：适合精细测量，但对相机分辨率要求高

#### 运行方式
```bash
./Projector_Crmera_test.exe
```

按提示选择功能，在实时预览中调节参数，按's'保存参数。

## 五、参数配置文件 camera_params.txt

### 文件格式
```ini
# 相机参数配置文件
exposureTimeUs=20000.0      # 曝光时间（微秒）
exposureAutoMode=0          # 自动曝光（0=关闭，1=开启）
gainValue=8.0               # 增益值（dB）
gainAutoMode=0              # 自动增益（0=关闭，1=开启）
frameRate=10.0              # 帧率（fps）
triggerDelayUs=0            # 触发延时（微秒）
enableChunkData=0           # 块数据（0=关闭，1=开启）
printCurrentParams=1        # 打印参数（0=关闭，1=开启）
```

### 参数范围和建议值
| 参数 | 范围 | 建议值 | 说明 |
|------|------|--------|------|
| exposureTimeUs | 20-100000 | 20000 | 曝光时间，影响图像亮度 |
| gainValue | 0-30 | 8.0 | 增益，影响图像亮度和噪声 |
| frameRate | 1-100 | 10.0 | 帧率，影响采集速度 |
| triggerDelayUs | 0-10000 | 0 | 触发延时，用于同步 |

## 六、常见问题与解决方案

### 6.1 投影仪相关问题

**Q: 投影仪连接失败**
```
A: 检查USB连接和驱动安装
   - 确认投影仪电源开启
   - 检查USB线缆连接
   - 安装对应型号的驱动程序
```

**Q: 投影图案频闪**
```
A: 优化投影模式和稳定时间
   - 使用project(true)启动连续模式
   - 调用pause()暂停在指定帧
   - 增加LED电流稳定时间
   - 使用完整的稳定流程（断开重连）
```

**Q: LED亮度不足**
```
A: 调整LED电流设置
   projector->setLEDCurrent(0.95, 0.95, 0.95);  // 提高到95%
   // 范围0.0-1.0，值越大越亮
```

### 6.2 相机相关问题

**Q: 相机图像过暗**
```
A: 调整曝光和增益参数
   exposureTimeUs=50000.0    // 增加曝光时间
   gainValue=12.0            // 增加增益值
```

**Q: 相机图像噪声大**
```
A: 降低增益，增加曝光时间
   exposureTimeUs=30000.0    // 增加曝光时间
   gainValue=6.0             // 降低增益值
```

**Q: 同步问题**
```
A: 调整触发延时
   triggerDelayUs=1000       // 增加1ms延时
   // 确保投影稳定后再触发相机
```

### 6.3 图像质量问题

**Q: 条纹对比度低**
```
A: 优化投影和采集参数
   - 提高LED电流到0.95
   - 调整相机曝光时间
   - 确保环境光线适中
   - 检查投影距离和角度
```

**Q: 图像模糊**
```
A: 检查焦距和曝光时间
   - 调整投影仪和相机焦距
   - 减少曝光时间避免运动模糊
   - 确保物体静止
```

## 七、最佳实践建议

### 7.1 参数调节流程
1. **初始设置**：运行`cameraTest.exe`获取基础参数
2. **实时调节**：使用`Projector_Crmera_test.exe`在实时预览中精细调节
3. **参数保存**：调节完成后按's'保存参数
4. **正式采集**：运行`projectorWithCreame.exe`进行完整采集

### 7.2 系统稳定性
- **投影仪预热**：连接后等待5-10秒让投影仪稳定
- **LED稳定**：设置LED电流后等待1秒以上
- **相机预热**：开始采集前让相机运行500ms
- **环境控制**：保持稳定的光照环境

### 7.3 图像质量优化
- **条纹频率**：根据测量精度要求选择15-25的中等频率
- **投影距离**：保持适当的投影距离，避免过近或过远
- **表面处理**：对于高反射表面可喷涂显影剂改善条纹质量

## 八、扩展开发指南

### 8.1 添加新的投影图案
```cpp
// 在generatePhaseShiftFringeImages函数中添加新图案
cv::Mat customPattern = generateCustomPattern(width, height, params);
patternSet.imgs_.push_back(customPattern);
```

### 8.2 自定义相机参数
```cpp
// 修改CameraParams结构体添加新参数
struct CameraParams {
    float customParam = 0.0f;  // 新参数
    // ... 其他参数
};
```

### 8.3 添加新的同步模式
```cpp
// 在ProjectorWithCamera.cpp中添加新的同步逻辑
bool customSyncCapture() {
    // 自定义同步采集逻辑
    projector->step();
    std::this_thread::sleep_for(std::chrono::milliseconds(customDelay));
    camera->trigger();
    return true;
}
```

## 九、版本更新记录

- **v1.0**：基础投影仪和相机控制功能
- **v1.1**：添加按键触发投影功能
- **v1.2**：优化投影稳定性，减少频闪
- **v1.3**：添加全白图案投影功能
- **v1.4**：改进参数调节界面，添加直方图显示
- **v1.5**：统一图像命名规则，优化同步逻辑

---

本文档涵盖了ProjectorWithCamera项目的完整使用方法，如有问题请参考源码注释或联系开发团队。