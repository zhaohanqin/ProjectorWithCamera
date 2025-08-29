// 投影仪与相机协作演示（相机部分以注释形式保留接口）
//
// 设计目标：
// - 提供一个可直接复用的“步进投影 + 软触发采集”的协作流程样例；
// - 相机与投影仪均采用“软触发”：由软件下发指令来推进投影以及触发相机采集；
// - 生成 N 步相移条纹，顺序：垂直 N 张 + 水平 N 张，总计 2N 张；
// - 每投影一张（step一次），便触发一次相机采集（以注释形式保留）；
// - 默认投影仪型号为 "DLP4710"（如需其他型号，可在函数参数中修改）。

#include "projectorFactory.h"
#include "projector.h"
#include "projectorDlpc34xx.h"
#include "projectorDlpc34xxDual.h"

#include <opencv2/core.hpp>
// 如需将回调数据转为图像保存，可按需引入：#include <opencv2/imgcodecs.hpp> / <opencv2/highgui.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <atomic>
#if defined(_WIN32)
#include <windows.h>
#endif

// ========== MVS 工业相机 SDK 头文件 ==========
#include "MvCameraControl.h"
// 图像保存
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>

namespace slmaster_demo {

// 生成N步相移条纹（先垂直N张，再水平N张），与 generate_fringe_patterns.py 参数一致
//
// 函数：generatePhaseShiftFringeImages
// 作用：
// - 生成 N 步相移条纹图像，先垂直 N 张，再水平 N 张（总数 2N），用于装载到投影仪；
// - 生成的图像为 CV_8UC1 灰度图，像素范围 [0,255]；
// 参数说明：
// - width     ：图像宽度，须与投影仪 DMD 宽度一致（如 DLP4710 为 1920）；
// - height    ：图像高度，须与投影仪 DMD 高度一致（如 DLP4710 为 1080）；
// - frequency ：条纹频率/周期数（整幅图中正弦条纹重复的周期数量）；
// - intensity ：条纹强度（振幅），最终灰度近似为 offset + intensity * sin(...)；
// - offset    ：亮度偏移（平均灰度/直流分量），确保图像处于有效动态范围；
// - noiseLevel：噪声水平（高斯噪声标准差），0 表示不加噪声；
// - steps     ：相移步数 N（返回图像总数为 2N）。
// 返回值：
// - 长度为 2N 的图像数组，前 N 张为垂直条纹，相位为 [0, 2π) 等步进；后 N 张为水平条纹，相位同理。
static std::vector<cv::Mat> generatePhaseShiftFringeImages(
    int width,
    int height,
    int frequency,
    int intensity,
    int offset,
    double noiseLevel,
    int steps
) 
{
    std::vector<cv::Mat> result;
    if (width <= 0 || height <= 0 || frequency <= 0 || steps <= 0) {
        return result;
    }

    if (intensity < 0) intensity = 0;
    if (intensity > 255) intensity = 255;
    if (offset < 0) offset = 0;
    if (offset > 255) offset = 255;

    result.reserve(static_cast<size_t>(steps * 2));

    const double twoPi = 2.0 * 3.14159265358979323846;
    const double stepPhase = twoPi / static_cast<double>(steps);

    // 垂直条纹（相位沿x变化）
    for (int p = 0; p < steps; ++p) {
        const double phase = static_cast<double>(p) * stepPhase;
        cv::Mat img(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            unsigned char* row = img.ptr<unsigned char>(y);
            for (int x = 0; x < width; ++x) {
                const double t = static_cast<double>(x) / static_cast<double>(width);
                double grayValue = static_cast<double>(offset) + static_cast<double>(intensity) *
                    std::sin(twoPi * static_cast<double>(frequency) * t + phase);
                if (noiseLevel > 0.0) {
                    grayValue += cv::theRNG().gaussian(noiseLevel);
                }
                int g = static_cast<int>(std::lround(grayValue));
                if (g < 0) g = 0; if (g > 255) g = 255;
                row[x] = static_cast<unsigned char>(g);
            }
        }
        result.push_back(img);
    }
    
    // 水平条纹（相位沿y变化）
    for (int p = 0; p < steps; ++p) {
        const double phase = static_cast<double>(p) * stepPhase;
        cv::Mat img(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const double t = static_cast<double>(y) / static_cast<double>(height);
            const double s = std::sin(twoPi * static_cast<double>(frequency) * t + phase);
            double lineGray = static_cast<double>(offset) + static_cast<double>(intensity) * s;
            if (noiseLevel > 0.0) {
                lineGray += cv::theRNG().gaussian(noiseLevel);
            }
            int lineGrayInt = static_cast<int>(std::lround(lineGray));
            if (lineGrayInt < 0) lineGrayInt = 0; if (lineGrayInt > 255) lineGrayInt = 255;
            const unsigned char v = static_cast<unsigned char>(lineGrayInt);
            unsigned char* row = img.ptr<unsigned char>(y);
            std::memset(row, static_cast<int>(v), static_cast<size_t>(width));
        }
        result.push_back(img);
    }

    return result;
}

// 演示：投影仪与相机协作（均为“软触发”）
// 流程：
// 1) 初始化并选择投影仪；
// 2) 生成 2N 张相移条纹（垂直 N + 水平 N），装载到投影仪；
// 3) 开始投影；循环 2N 次：
//    - 先软件步进 projector->step() 使投影仪显示下一张图；
//    - 等待稳定时间；
//    - 再以软件触发相机采集一帧（此处保留注释接口）；
// 4) 全部完成后停止投影并断开连接。
// 参数说明：
// - projectorModel：投影仪型号字符串，默认 "DLP4710"；
// - deviceWidth   ：投影分辨率宽度（需与设备一致，如 1920）；
// - deviceHeight  ：投影分辨率高度（需与设备一致，如 1080）；
// - steps         ：相移步数 N；
// - frequency     ：条纹频率/周期数；
// - intensity     ：条纹强度/振幅（建议不超过 127）；
// - offset        ：亮度偏移（建议位于 128 附近以居中）；
// - noiseStd      ：噪声标准差（0 表示不加噪声）。
// 返回值：
// - true 表示流程执行成功；false 表示中途失败。
bool runProjectorCameraCooperation(
    const std::string& projectorModel /* = "DLP4710" */,
    int deviceWidth,
    int deviceHeight,
    int steps,
    int frequency,
    int intensity,
    int offset,
    double noiseStd,
    const std::string& cameraSerial /* = "NULL" 表示自动选择第一台 */,
    const std::string& outputDir /* 为空表示当前目录 */,
    double exposureTimeUs /* 曝光时间(微秒) */,
    double gainValue /* 增益 */,
    double acquisitionFps /* 采集帧率 */,
    double triggerDelayUs /* 触发延时(微秒) */
) {
    using namespace slmaster::device;

    try {
        // ========== 1) 初始化与选择投影仪 ==========
        slmaster::device::ProjectorFactory factory;
        Projector* projector = factory.getProjector(projectorModel);
        if (projector == nullptr) {
            std::cerr << u8"获取投影仪失败，型号: " << projectorModel << std::endl;
            return false;
        }
        if (!projector->connect()) {
            std::cerr << u8"连接投影仪失败" << std::endl;
            return false;
        }
        std::cout << u8"投影仪已连接: " << projectorModel << std::endl;

        // ========== 1.5) 初始化相机（MVS，软触发） ==========
        MV_CC_DEVICE_INFO_LIST deviceList{};
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
        if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
            std::cerr << u8"未发现可用相机，错误码: " << nRet << std::endl;
            projector->disConnect();
            return false;
        }

        MV_CC_DEVICE_INFO* pSelectedDevice = nullptr;
        if (!cameraSerial.empty() && cameraSerial != "NULL") {
            for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
                MV_CC_DEVICE_INFO* pInfo = deviceList.pDeviceInfo[i];
                const char* serial = nullptr;
                // 根据传输层类型选择序列号字段（USB 或 GigE）
                if (pInfo->nTLayerType == MV_USB_DEVICE) {
                    serial = (const char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
                } else if (pInfo->nTLayerType == MV_GIGE_DEVICE) {
                    serial = (const char*)pInfo->SpecialInfo.stGigEInfo.chSerialNumber;
                }
                if (serial && cameraSerial == serial) {
                    pSelectedDevice = pInfo;
                    break;
                }
            }
            if (pSelectedDevice == nullptr) {
                std::cerr << u8"未找到匹配序列号的相机，序列号: " << cameraSerial << u8"，将使用第一台设备" << std::endl;
            }
        }
        if (pSelectedDevice == nullptr) {
            pSelectedDevice = deviceList.pDeviceInfo[0];
        }
        void* cameraHandle = nullptr;
        nRet = MV_CC_CreateHandle(&cameraHandle, pSelectedDevice);
        if (nRet != MV_OK || cameraHandle == nullptr) {
            std::cerr << u8"创建相机句柄失败，错误码: " << nRet << std::endl;
            projector->disConnect();
            return false;
        }

        nRet = MV_CC_OpenDevice(cameraHandle);
        if (nRet != MV_OK) {
            std::cerr << u8"打开相机失败，错误码: " << nRet << std::endl;
            MV_CC_DestroyHandle(cameraHandle);
            projector->disConnect();
            return false;
        }

        // 更稳健的触发配置：优先按字符串设置，兼容不同固件枚举
        MV_CC_SetEnumValueByString(cameraHandle, "PixelFormat", "Mono8");
        MV_CC_SetEnumValueByString(cameraHandle, "TriggerSelector", "FrameStart");
        MV_CC_SetEnumValue(cameraHandle, "TriggerMode", 1);
        MV_CC_SetEnumValueByString(cameraHandle, "TriggerSource", "Software");
        MV_CC_SetEnumValueByString(cameraHandle, "AcquisitionMode", "Continuous");

        // ========== 相机参数：曝光/增益/帧率/触发延时 ==========
        // 说明（对协作采集的影响）：
        // - ExposureTime(us)：决定单帧积分时间；
        //   * 增大：图像更亮，但投影步进周期需更长，运动模糊风险增加；
        //   * 减小：图像更暗，但可更快步进与触发；
        //   在结构光中，应确保曝光窗口覆盖“投影稳定区间”。
        // - Gain：提升亮度但同时放大噪声；尽量用曝光满足亮度，增益只作补偿。
        // - AcquisitionFrameRate：指导相机采集速率；
        //   在软触发下通常作为上限，需要与“曝光+读出+传输”匹配，避免阻塞/丢帧。
        // - TriggerDelay(us)：触发到曝光开始的延时；
        //   常用于对齐 projector->step() 之后的稳定时间，避免采到过渡帧。
        // 参数策略：仅当用户提供非负值时才关闭对应自动模式并设置；否则保持自动。
        // 仅当用户指定参数时才关闭自动曝光/增益并手动设置
        if (exposureTimeUs >= 0.0) {
            MV_CC_SetEnumValue(cameraHandle, "ExposureAuto", 0);
        }
        if (gainValue >= 0.0) {
            MV_CC_SetEnumValue(cameraHandle, "GainAuto", 0);
        }

        // 曝光时间
        if (exposureTimeUs > 0.0) {
            MVCC_FLOATVALUE expRange{};
            if (MV_CC_GetFloatValue(cameraHandle, "ExposureTime", &expRange) == MV_OK) {
                double v = exposureTimeUs;
                if (v < expRange.fMin) v = expRange.fMin;
                if (v > expRange.fMax) v = expRange.fMax;
                MV_CC_SetFloatValue(cameraHandle, "ExposureTime", v);
                std::cout << "ExposureTime(us): " << v << " [" << expRange.fMin << ", " << expRange.fMax << "]" << std::endl;
            }
        }
        // 增益
        if (gainValue > 0.0) {
            MVCC_FLOATVALUE gRange{};
            if (MV_CC_GetFloatValue(cameraHandle, "Gain", &gRange) == MV_OK) {
                double v = gainValue;
                if (v < gRange.fMin) v = gRange.fMin;
                if (v > gRange.fMax) v = gRange.fMax;
                MV_CC_SetFloatValue(cameraHandle, "Gain", v);
                std::cout << "Gain: " << v << " [" << gRange.fMin << ", " << gRange.fMax << "]" << std::endl;
            }
        }
        // 帧率
        if (acquisitionFps > 0.0) {
            MV_CC_SetBoolValue(cameraHandle, "AcquisitionFrameRateEnable", true);
            MVCC_FLOATVALUE fRange{};
            if (MV_CC_GetFloatValue(cameraHandle, "AcquisitionFrameRate", &fRange) == MV_OK) {
                double v = acquisitionFps;
                if (v < fRange.fMin) v = fRange.fMin;
                if (v > fRange.fMax) v = fRange.fMax;
                MV_CC_SetFloatValue(cameraHandle, "AcquisitionFrameRate", v);
                std::cout << "AcquisitionFrameRate: " << v << " [" << fRange.fMin << ", " << fRange.fMax << "]" << std::endl;
            }
        }
        // 触发延时
        if (triggerDelayUs >= 0.0) {
            MVCC_FLOATVALUE dRange{};
            if (MV_CC_GetFloatValue(cameraHandle, "TriggerDelay", &dRange) == MV_OK) {
                double v = triggerDelayUs;
                if (v < dRange.fMin) v = dRange.fMin;
                if (v > dRange.fMax) v = dRange.fMax;
                MV_CC_SetFloatValue(cameraHandle, "TriggerDelay", v);
                std::cout << "TriggerDelay(us): " << v << " [" << dRange.fMin << ", " << dRange.fMax << "]" << std::endl;
            }
        }

        // 为回调准备上下文（线程安全计数器 + 保存目录 + 总帧数）
        struct CallbackContext {
            std::atomic<int> frameIndex{0};
            int totalFrames{0};
            std::string saveDir;
            int stepsPerOrientation{0};
        } cbCtx;
        cbCtx.totalFrames = steps * 2;
        cbCtx.stepsPerOrientation = steps;
        if (!outputDir.empty()) {
            cbCtx.saveDir = outputDir;
        } else {
            // 默认保存到当前目录下 images 子目录
            cbCtx.saveDir = (std::filesystem::current_path() / "images").string();
        }
        // 确保目录存在
        try { std::filesystem::create_directories(cbCtx.saveDir); } catch (...) {}

        // 相机的回调：打印帧信息并保存为 I001_V.png / I005_H.png
        auto ImageCallbackEx = [](unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
			if (!pFrameInfo || !pUser) return;// 安全检查, 避免野指针访问
            CallbackContext* ctx = reinterpret_cast<CallbackContext*>(pUser);
            const int idx = 1 + ctx->frameIndex.fetch_add(1); // 从1开始
            std::cout << "Get One Frame: Width[" << pFrameInfo->nWidth
                      << "] Height[" << pFrameInfo->nHeight
                      << "] Index[" << idx << "/" << ctx->totalFrames << "]" << std::endl;
            // 仅保存前 totalFrames 张，避免多余帧命名越界
            if (idx <= ctx->totalFrames) {
                cv::Mat img(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
                cv::Mat imgCopy = img.clone(); // 复制缓冲，避免回调后内存复用
                const bool isVertical = idx <= ctx->stepsPerOrientation;
                const char orient = isVertical ? 'V' : 'H';
                char nameBuf[32];
                std::snprintf(nameBuf, sizeof(nameBuf), "I%03d_%c.png", idx, orient);
                std::string filename = nameBuf;
                std::string filepath = (std::filesystem::path(ctx->saveDir) / filename).string();
                try {
                    cv::imwrite(filepath, imgCopy);
                } catch (...) {
                    std::cerr << u8"保存图像失败: " << filepath << std::endl;
                }
            }
        };

        nRet = MV_CC_RegisterImageCallBackEx(cameraHandle, ImageCallbackEx, &cbCtx);
        if (nRet != MV_OK) {
            std::cerr << u8"注册图像回调失败，错误码: " << nRet << std::endl;
            MV_CC_CloseDevice(cameraHandle);
            MV_CC_DestroyHandle(cameraHandle);
            projector->disConnect();
            return false;
        }

        nRet = MV_CC_StartGrabbing(cameraHandle);
        if (nRet != MV_OK) {
            std::cerr << u8"开始采集失败，错误码: " << nRet << std::endl;
            MV_CC_CloseDevice(cameraHandle);
            MV_CC_DestroyHandle(cameraHandle);
            projector->disConnect();
            return false;
        }

        // ========== 2) 生成相移条纹 ==========
        auto imgs = generatePhaseShiftFringeImages(deviceWidth, deviceHeight,
            frequency, intensity, offset, noiseStd, steps);
        if (static_cast<int>(imgs.size()) != steps * 2) {
            std::cerr << u8"生成条纹图像数量异常: " << imgs.size() << std::endl;
            MV_CC_StopGrabbing(cameraHandle);
            MV_CC_CloseDevice(cameraHandle);
            MV_CC_DestroyHandle(cameraHandle);
            projector->disConnect();
            return false;
        }

        std::vector<PatternOrderSet> patternSets(2);

        // 垂直条纹配置（前N张）
        patternSets[0].exposureTime_ = 4000;      // 可按需外部传入
        patternSets[0].preExposureTime_ = 3000;
        patternSets[0].postExposureTime_ = 3000;
        patternSets[0].illumination_ = Blue;
        patternSets[0].invertPatterns_ = false;
        patternSets[0].isVertical_ = true;
        patternSets[0].isOneBit_ = false;
        patternSets[0].patternArrayCounts_ = deviceWidth;
        patternSets[0].imgs_.assign(imgs.begin(), imgs.begin() + steps);

        // 水平条纹配置（后N张）
        patternSets[1].exposureTime_ = 4000;
        patternSets[1].preExposureTime_ = 3000;
        patternSets[1].postExposureTime_ = 3000;
        patternSets[1].illumination_ = Blue;
        patternSets[1].invertPatterns_ = false;
        patternSets[1].isVertical_ = false;
        patternSets[1].isOneBit_ = false;
        patternSets[1].patternArrayCounts_ = deviceHeight;
        patternSets[1].imgs_.assign(imgs.begin() + steps, imgs.end());

        if (!projector->populatePatternTableData(patternSets)) {
            std::cerr << u8"装载图案表失败" << std::endl;
            // 清理相机与投影仪资源
            MV_CC_StopGrabbing(cameraHandle);
            MV_CC_CloseDevice(cameraHandle);
            MV_CC_DestroyHandle(cameraHandle);
            projector->disConnect();
            return false;
        }

        // ========== 3) 开始投影 + 相机协作（软触发顺序：先投影，后采集） ==========
        // 步进模式启动（与 ProjectorTest 一致），每次 step() 前进一步
        if (!projector->project(false)) {
            std::cerr << u8"启动投影（步进模式）失败" << std::endl;
            MV_CC_StopGrabbing(cameraHandle);
            MV_CC_CloseDevice(cameraHandle);
            MV_CC_DestroyHandle(cameraHandle);
            projector->disConnect();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // 软触发顺序：先步进，按照时序预算等待，再触发相机
        auto waitMsFromTiming = [&](bool vertical){
            int pre = vertical ? patternSets[0].preExposureTime_ : patternSets[1].preExposureTime_;
            int exp = vertical ? patternSets[0].exposureTime_     : patternSets[1].exposureTime_;
            int post= vertical ? patternSets[0].postExposureTime_ : patternSets[1].postExposureTime_;
            int ms = (pre + exp + post) / 1000;
            if (ms < 1) ms = 1;
            return ms + 10; // 余量
        };
        // 按固定次数步进，避免依赖会在跨 PatternSet 重置的设备状态计数
        for (int i = 0; i < steps * 2; ++i) {
            if (!projector->step()) {
                std::cerr << u8"步进第" << i << u8"帧失败" << std::endl;
                break;
            }
            const bool isVerticalNow = (i < steps);
            const int waitMs = waitMsFromTiming(isVerticalNow);
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));

            // 再触发相机（软件触发）
            MV_CC_SetEnumValueByString(cameraHandle, "AcquisitionMode", "Continuous");
            int tRet = MV_CC_TriggerSoftwareExecute(cameraHandle);
            if (tRet != MV_OK) {
                std::cerr << u8"第" << i << u8"次软件触发失败，错误码: " << tRet << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
        }

		// ========== 4) 全部完成，停止投影与断开连接 ==========
        if (!projector->stop()) {
            std::cerr << u8"停止投影失败" << std::endl;
        }
        // 关闭相机
        MV_CC_StopGrabbing(cameraHandle);
        MV_CC_CloseDevice(cameraHandle);
        MV_CC_DestroyHandle(cameraHandle);
        projector->disConnect();
        std::cout << u8"投影与相机协作流程完成" << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return false;
    }
}

} // namespace slmaster_demo

// 示例调用（可由外部单元测试或GUI事件触发）
 int main() {
#if defined(_WIN32)
     // 将控制台输入/输出代码页切换为 UTF-8，避免中文乱码
     SetConsoleOutputCP(CP_UTF8);
     SetConsoleCP(CP_UTF8);
#endif
     // 示例：相机序列号与保存目录可按需填写；序列号传 "NULL" 表示自动选择第一台
     const std::string cameraSerial = "NULL"; // 或者例如 "DA1015150"
     const std::string saveDir = "";         // 为空表示当前目录
     // 相机参数示例：默认 -1 表示未指定，保持自动/默认；填写非负数则关闭自动并按值设置
     // 参数含义：
     // - exposureUs：曝光时间(微秒)；画面亮度↑/动态模糊↑/步进周期需更长
     // - gain：增益；亮度↑/噪声↑，尽量低增益
     // - fps：采集帧率；需与“曝光+读出+传输”匹配
     // - trigDelayUs：触发延时(微秒)；与投影稳定时刻对齐
     const double exposureUs = -1.0;   // 例如 8000.0 表示 8ms
     const double gain = -1.0;         // 例如 8.0
     const double fps = -1.0;          // 例如 60.0
     const double trigDelayUs = -1.0;  // 例如 1000.0 表示 1ms 触发延时

     slmaster_demo::runProjectorCameraCooperation(
         "DLP4710",   // 选择型号
         1920, 1080,   // 设备分辨率
         4,            // 相移步数N
         32,           // 条纹频率
         100,          // 强度
         128,          // 偏移
         0.0,          // 噪声
         cameraSerial,
         saveDir,
         exposureUs,
         gain,
         fps,
         trigDelayUs
     );
 }


