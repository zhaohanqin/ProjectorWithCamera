// 相机连接与拍摄测试（基于海康 MVS C++ SDK）
// 功能：
// 1) 枚举相机并按序列号选择；若传 "NULL" 或未指定则默认第一台
// 2) 打开设备、配置为软件触发 Mono8
// 3) 注册回调并软触发抓拍 N 张图像，保存为 I1..IN.png 到指定目录（或当前目录）

#include "MvCameraControl.h"

#include <iostream>
#include <string>
#include <atomic>
#include <filesystem>
#include <thread>
#include <chrono>
// 为使用 std::min/std::max 添加头文件
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#if defined(_WIN32)
// 避免 Windows 宏 min/max 与 std::min/std::max 冲突
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

// 简易测试框架（与 ProjectorTest.cpp 风格一致）
struct TestResults {
    int totalTests = 0;
    int passedTests = 0;
    int failedTests = 0;

    void printSummary() const {
        std::cout << "\n=== 相机测试结果汇总 ===" << std::endl;
        std::cout << "总测试数: " << totalTests << std::endl;
        std::cout << "通过测试: " << passedTests << std::endl;
        std::cout << "失败测试: " << failedTests << std::endl;
        std::cout << "成功率: " << (totalTests > 0 ? (passedTests * 100.0 / totalTests) : 0) << "%" << std::endl;
    }
};

static TestResults g_camTestResults;

static void assertTrue(bool condition, const std::string& message) {
    g_camTestResults.totalTests++;
    if (condition) {
        g_camTestResults.passedTests++;
        std::cout << "[PASS] " << message << std::endl;
    } else {
        g_camTestResults.failedTests++;
        std::cout << "[FAIL] " << message << std::endl;
    }
}

static void assertNotNull(const void* ptr, const std::string& message) {
    g_camTestResults.totalTests++;
    if (ptr != nullptr) {
        g_camTestResults.passedTests++;
        std::cout << "[PASS] " << message << std::endl;
    } else {
        g_camTestResults.failedTests++;
        std::cout << "[FAIL] " << message << std::endl;
    }
}

// 回调上下文：用于跨回调传递保存状态
// - frameIndex: 线程安全的帧计数器，从0开始计数
// - totalFrames: 期望保存的总帧数，用于限制命名为 I1..I(总数)
// - saveDir: 图像保存目录的绝对或相对路径
struct CallbackContext {
    std::atomic<int> frameIndex{0};
    int totalFrames{0};
    std::string saveDir;
};

// 运行相机连接与图像采集测试
// 参数：
// - cameraSerial: 指定要连接的相机序列号；传 "NULL" 或空字符串表示自动选择第一台
// - outputDir: 图像保存目录；为空表示使用当前工作目录
// - framesToCapture: 软触发采集并保存的帧数（将保存为 I1..IframesToCapture.png）
// 返回：
// - true 表示测试流程全部成功；false 表示其中出现错误
// 额外可选参数：
// - exposureTimeUs/gainValue/acquisitionFps/triggerDelayUs 传负数表示“未指定”，保持当前相机模式（不关闭自动）
static bool runCameraTest(const std::string& cameraSerial,
                          const std::string& outputDir,
                          int framesToCapture,
                          double exposureTimeUs,
                          double gainValue,
                          double acquisitionFps,
                          double triggerDelayUs) {
    // 1) 枚举设备
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        std::cerr << "未发现可用相机，错误码: " << nRet << std::endl;
        return false;
    }

    // 2) 选择设备（优先匹配序列号；否则退回第一台）
    MV_CC_DEVICE_INFO* pSelectedDevice = nullptr;
    if (!cameraSerial.empty() && cameraSerial != "NULL")//如果相机序列号不为空，则根据序列号选择相机
    {
        for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO* pInfo = deviceList.pDeviceInfo[i];
            const char* serial = nullptr;
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
            std::cerr << "未找到匹配序列号的相机，使用第一台。序列号: " << cameraSerial << std::endl;
        }
    }
    if (pSelectedDevice == nullptr) {//如果未找到匹配序列号的相机，则使用第一台
        pSelectedDevice = deviceList.pDeviceInfo[0];
    }

    // 3) 创建并打开设备
    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, pSelectedDevice);//创建相机句柄
    if (nRet != MV_OK || handle == nullptr) {
        std::cerr << "创建相机句柄失败，错误码: " << nRet << std::endl;
        return false;
    }
    nRet = MV_CC_OpenDevice(handle);//打开相机
    if (nRet != MV_OK) {
        std::cerr << "打开相机失败，错误码: " << nRet << std::endl;
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 4) 基础配置：Mono8 + 软件触发
    // 说明：
    // - PixelFormat 0x02180014 为 Mono8（单通道8位），便于直接保存为灰度图
    // - TriggerMode 为 1 表示开启触发模式
    // - TriggerSource 为 7 表示使用软件触发
    // 更稳健的配置：优先使用 ByString，避免不同设备枚举值差异
    MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    MV_CC_SetEnumValueByString(handle, "TriggerSelector", "FrameStart");
    MV_CC_SetEnumValue(handle, "TriggerMode", 1);          // On
    MV_CC_SetEnumValueByString(handle, "TriggerSource", "Software");
    MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous");

    // 4.1) 相机参数：仅当用户提供值时才关闭自动并设置；否则保持当前自动/默认
    // 参数含义与影响（简要）：
    // - ExposureTime (us, 曝光时间)：
    //   作用：决定单帧积分时间，数值越大，亮度越高；
    //   影响：
    //     * 增大：画面更亮、动态模糊风险更高、帧率上限下降；
    //     * 减小：画面更暗、运动更清晰、可实现更高触发频率；
    //   提示：在结构光/步进投影中，曝光时间应与投影曝光窗口匹配，以避免截断或过曝。
    // - Gain (增益)：
    //   作用：放大信号强度；
    //   影响：
    //     * 增大：亮度提升、同时噪声放大、SNR 下降、量化噪声更明显；
    //     * 减小：亮度下降、噪声更低；
    //   提示：优先通过曝光提升亮度，增益仅作为补偿手段。
    // - AcquisitionFrameRate (采集帧率)：
    //   作用：限制或指导采集速率（触发模式下通常是上限/定时参考）；
    //   影响：
    //     * 提高：需要更短曝光、更高传输带宽；
    //     * 降低：留给曝光与读出更多时间；
    //   提示：在软触发/硬触发时需确保曝光+读出时间小于帧周期，否则会丢帧或阻塞。
    // - TriggerDelay (us, 触发延时)：
    //   作用：从接收触发到开始曝光的延时；
    //   影响：
    //     * 用于对齐外部事件（如投影仪步进后稳定时间）；
    //   提示：结构光中常设置为“投影步进到稳定的等待时间”，避免抓到过渡帧。
    if (exposureTimeUs >= 0.0) {
        MV_CC_SetEnumValue(handle, "ExposureAuto", 0); // 0=Off
        MVCC_FLOATVALUE exposureRange{};
        if (MV_CC_GetFloatValue(handle, "ExposureTime", &exposureRange) == MV_OK) {
            double v = exposureTimeUs;
            if (v < exposureRange.fMin) v = exposureRange.fMin;
            if (v > exposureRange.fMax) v = exposureRange.fMax;
            MV_CC_SetFloatValue(handle, "ExposureTime", v);
            std::cout << "ExposureTime(us): " << v << " [" << exposureRange.fMin << ", " << exposureRange.fMax << "]" << std::endl;
        }
    }

    if (gainValue >= 0.0) {
        MV_CC_SetEnumValue(handle, "GainAuto", 0);
        MVCC_FLOATVALUE gainRange{};
        if (MV_CC_GetFloatValue(handle, "Gain", &gainRange) == MV_OK) {
            double v = gainValue;
            if (v < gainRange.fMin) v = gainRange.fMin;
            if (v > gainRange.fMax) v = gainRange.fMax;
            MV_CC_SetFloatValue(handle, "Gain", v);
            std::cout << "Gain: " << v << " [" << gainRange.fMin << ", " << gainRange.fMax << "]" << std::endl;
        }
    }

    if (acquisitionFps >= 0.0) {
        MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
        MVCC_FLOATVALUE fpsRange{};
        if (MV_CC_GetFloatValue(handle, "AcquisitionFrameRate", &fpsRange) == MV_OK) {
            double v = acquisitionFps;
            if (v < fpsRange.fMin) v = fpsRange.fMin;
            if (v > fpsRange.fMax) v = fpsRange.fMax;
            MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", v);
            std::cout << "AcquisitionFrameRate: " << v << " [" << fpsRange.fMin << ", " << fpsRange.fMax << "]" << std::endl;
        }
    }

    if (triggerDelayUs >= 0.0) {
        MVCC_FLOATVALUE tdelayRange{};
        if (MV_CC_GetFloatValue(handle, "TriggerDelay", &tdelayRange) == MV_OK) {
            double v = triggerDelayUs;
            if (v < tdelayRange.fMin) v = tdelayRange.fMin;
            if (v > tdelayRange.fMax) v = tdelayRange.fMax;
            MV_CC_SetFloatValue(handle, "TriggerDelay", v);
            std::cout << "TriggerDelay(us): " << v << " [" << tdelayRange.fMin << ", " << tdelayRange.fMax << "]" << std::endl;
        }
    }

    // 5) 准备保存目录与回调
    // - 创建保存目录（若不存在）
    // - 注册图像回调：接收帧数据并保存为 I1..I(总数).png
    CallbackContext ctx;
    ctx.totalFrames = framesToCapture;
    if (!outputDir.empty()) {
        ctx.saveDir = outputDir;
    } else {
        // 默认保存到当前目录下的 images 子目录
        ctx.saveDir = (std::filesystem::current_path() / "images").string();
    }
    try { std::filesystem::create_directories(ctx.saveDir); } catch (...) {}

    // 图像回调：将采集到的帧以灰度图保存
    // 注意：SDK在回调中提供的缓冲区在返回后可能失效，本处直接在回调中同步写盘
    auto ImageCallbackEx = [](unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
        if (!pFrameInfo || !pUser) return;
        CallbackContext* c = reinterpret_cast<CallbackContext*>(pUser);
        const int idx = 1 + c->frameIndex.fetch_add(1);
        std::cout << "Get One Frame: W[" << pFrameInfo->nWidth
                  << "] H[" << pFrameInfo->nHeight
                  << "] Index[" << idx << "/" << c->totalFrames << "]" << std::endl;
        if (idx <= c->totalFrames) {
            cv::Mat img(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
            cv::Mat imgCopy = img.clone(); // 复制一份，避免回调返回后缓冲区失效
            std::string filename = "I" + std::to_string(idx) + ".png";
            std::string path = (std::filesystem::path(c->saveDir) / filename).string();
            try { cv::imwrite(path, imgCopy); } catch (...) { std::cerr << "保存失败: " << path << std::endl; }
        }
    };

    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallbackEx, &ctx);
    if (nRet != MV_OK) {
        std::cerr << "注册回调失败，错误码: " << nRet << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        std::cerr << "开始采集失败，错误码: " << nRet << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 6) 软触发抓拍 N 张
    // 确保处于连续采集模式
    MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous");
    // 每次触发后稍作等待，给数据传输与回调写盘留出时间
    for (int i = 0; i < framesToCapture; ++i) {
        int t = MV_CC_TriggerSoftwareExecute(handle);
        if (t != MV_OK) {
            std::cerr << "软件触发失败(第" << i << ")，错误码: " << t << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    // 简单等待回调写盘完成
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 7) 关闭与清理
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    std::cout << "相机测试完成，保存目录: " << ctx.saveDir << std::endl;
    return true;
}

// ==================== 拆分成与 ProjectorTest.cpp 类似的测试用例 ====================

// 设备枚举测试
static void testCameraEnumerate() {
    std::cout << "\n--- 测试相机枚举 ---" << std::endl;
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    bool ok = (nRet == MV_OK && deviceList.nDeviceNum > 0);
    if (ok) {
        std::cout << "发现相机数量: " << deviceList.nDeviceNum << std::endl;
    } else {
        std::cout << "未发现可用相机，错误码: " << nRet << std::endl;
    }
    assertTrue(ok, "能够成功枚举到至少一台相机");
}

// 打开/关闭设备测试（默认第一台或指定序列号）
static void testCameraOpenClose(const std::string& cameraSerial) {
    std::cout << "\n--- 测试相机打开/关闭 ---" << std::endl;
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        assertTrue(false, "设备枚举失败，无法进行打开/关闭测试");
        return;
    }
    MV_CC_DEVICE_INFO* pSelectedDevice = nullptr;
    if (!cameraSerial.empty() && cameraSerial != "NULL") {
        for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO* pInfo = deviceList.pDeviceInfo[i];
            const char* serial = nullptr;
            if (pInfo->nTLayerType == MV_USB_DEVICE) serial = (const char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
            else if (pInfo->nTLayerType == MV_GIGE_DEVICE) serial = (const char*)pInfo->SpecialInfo.stGigEInfo.chSerialNumber;
            if (serial && cameraSerial == serial) { pSelectedDevice = pInfo; break; }
        }
    }
    if (!pSelectedDevice) pSelectedDevice = deviceList.pDeviceInfo[0];

    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, pSelectedDevice);
    assertTrue(nRet == MV_OK && handle != nullptr, "创建相机句柄成功");
    if (nRet != MV_OK || handle == nullptr) return;
    nRet = MV_CC_OpenDevice(handle);
    assertTrue(nRet == MV_OK, "打开相机成功");
    if (nRet == MV_OK) {
        MV_CC_CloseDevice(handle);
    }
    MV_CC_DestroyHandle(handle);
}

// 配置像素格式与触发模式测试
static void testCameraConfigureTrigger(const std::string& cameraSerial) {
    std::cout << "\n--- 测试相机配置（Mono8 + 软件触发）---" << std::endl;
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        assertTrue(false, "设备枚举失败，无法进行配置测试");
        return;
    }
    MV_CC_DEVICE_INFO* pSelectedDevice = nullptr;
    if (!cameraSerial.empty() && cameraSerial != "NULL") {
        for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO* pInfo = deviceList.pDeviceInfo[i];
            const char* serial = nullptr;
            if (pInfo->nTLayerType == MV_USB_DEVICE) serial = (const char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
            else if (pInfo->nTLayerType == MV_GIGE_DEVICE) serial = (const char*)pInfo->SpecialInfo.stGigEInfo.chSerialNumber;
            if (serial && cameraSerial == serial) { pSelectedDevice = pInfo; break; }
        }
    }
    if (!pSelectedDevice) pSelectedDevice = deviceList.pDeviceInfo[0];

    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, pSelectedDevice);
    if (nRet != MV_OK || !handle) { assertTrue(false, "创建句柄失败"); return; }
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) { assertTrue(false, "打开相机失败"); MV_CC_DestroyHandle(handle); return; }

    bool ok = true;
    // 按照工业相机指导文档的正确配置顺序
    ok = ok && (MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8") == MV_OK);
    ok = ok && (MV_CC_SetEnumValueByString(handle, "TriggerSelector", "FrameStart") == MV_OK);
    ok = ok && (MV_CC_SetEnumValue(handle, "TriggerMode", 1) == MV_OK);  // On
    ok = ok && (MV_CC_SetEnumValueByString(handle, "TriggerSource", "Software") == MV_OK);
    ok = ok && (MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous") == MV_OK);
    assertTrue(ok, "成功配置 Mono8 + 软件触发");

    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
}

// 参数设置测试（若设备支持则设置到允许范围内的安全值）
static void testCameraParameterTuning(const std::string& cameraSerial) {
    std::cout << "\n--- 测试相机参数设置（曝光/增益/帧率/触发延时）---" << std::endl;
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) { assertTrue(false, "设备枚举失败"); return; }
    MV_CC_DEVICE_INFO* pSelectedDevice = nullptr;
    if (!cameraSerial.empty() && cameraSerial != "NULL") {
        for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO* pInfo = deviceList.pDeviceInfo[i];
            const char* serial = nullptr;
            if (pInfo->nTLayerType == MV_USB_DEVICE) serial = (const char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
            else if (pInfo->nTLayerType == MV_GIGE_DEVICE) serial = (const char*)pInfo->SpecialInfo.stGigEInfo.chSerialNumber;
            if (serial && cameraSerial == serial) { pSelectedDevice = pInfo; break; }
        }
    }
    if (!pSelectedDevice) pSelectedDevice = deviceList.pDeviceInfo[0];

    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, pSelectedDevice);
    if (nRet != MV_OK || !handle) { assertTrue(false, "创建句柄失败"); return; }
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) { assertTrue(false, "打开相机失败"); MV_CC_DestroyHandle(handle); return; }

    bool ok = true;
    // 关闭自动曝光/增益
    MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
    MV_CC_SetEnumValue(handle, "GainAuto", 0);

    MVCC_FLOATVALUE exposureRange{}; if (MV_CC_GetFloatValue(handle, "ExposureTime", &exposureRange) == MV_OK) {
        const double vMin = static_cast<double>(exposureRange.fMin);
        const double vMax = static_cast<double>(exposureRange.fMax);
        const double vMid = (vMin + vMax) * 0.5;
        const double v = std::clamp(vMid, vMin, vMax);
        ok = ok && (MV_CC_SetFloatValue(handle, "ExposureTime", v) == MV_OK);
    }
    MVCC_FLOATVALUE gainRange{}; if (MV_CC_GetFloatValue(handle, "Gain", &gainRange) == MV_OK) {
        const double vMin = static_cast<double>(gainRange.fMin);
        const double vMax = static_cast<double>(gainRange.fMax);
        const double vTarget = (vMin + vMax) * 0.2;
        const double v = std::clamp(vTarget, vMin, vMax);
        ok = ok && (MV_CC_SetFloatValue(handle, "Gain", v) == MV_OK);
    }
    MVCC_FLOATVALUE fpsRange{}; if (MV_CC_GetFloatValue(handle, "AcquisitionFrameRate", &fpsRange) == MV_OK) {
        MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
        const double vMin = static_cast<double>(fpsRange.fMin);
        const double vMax = static_cast<double>(fpsRange.fMax);
        const double vTarget = (vMin + vMax) * 0.5;
        const double v = std::clamp(vTarget, vMin, vMax);
        ok = ok && (MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", v) == MV_OK);
    }
    MVCC_FLOATVALUE tdelayRange{}; if (MV_CC_GetFloatValue(handle, "TriggerDelay", &tdelayRange) == MV_OK) {
        const double vMin = static_cast<double>(tdelayRange.fMin);
        const double vMax = static_cast<double>(tdelayRange.fMax);
        const double v = std::clamp(vMin, vMin, vMax); // 取最小值
        ok = ok && (MV_CC_SetFloatValue(handle, "TriggerDelay", v) == MV_OK);
    }

    assertTrue(ok, "成功设置关键参数到有效范围");

    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
}

// 采集保存测试（复用端到端流程）
static void testCameraCaptureAndSave(const std::string& cameraSerial, const std::string& saveDir, int frames) {
    std::cout << "\n--- 测试相机采集并保存图像 ---" << std::endl;
    bool ok = runCameraTest(cameraSerial, saveDir, frames, -1.0, -1.0, -1.0, -1.0);
    assertTrue(ok, "端到端采集与保存成功");
}

// 完整端到端测试（可指定参数）
static void testCameraEndToEnd(const std::string& cameraSerial,
                               const std::string& saveDir,
                               int frames,
                               double exposureUs,
                               double gain,
                               double fps,
                               double trigDelayUs) {
    std::cout << "\n--- 测试相机端到端流程（可设参数）---" << std::endl;
    bool ok = runCameraTest(cameraSerial, saveDir, frames, exposureUs, gain, fps, trigDelayUs);
    assertTrue(ok, "端到端流程执行成功");
}

static void runAllCameraTests() {
    // 基础信息
    const std::string cameraSerial = "NULL"; // 或指定序列号，例如 "DA1015150"
    const std::string saveDir = "";

    // 基础能力
    testCameraEnumerate();
    testCameraOpenClose(cameraSerial);
    testCameraConfigureTrigger(cameraSerial);
    testCameraParameterTuning(cameraSerial);

    // 采集能力
    testCameraCaptureAndSave(cameraSerial, saveDir, 4);

    // 端到端示例（如需调参，可在此设置非负数）
    testCameraEndToEnd(cameraSerial, saveDir, 6, -1.0, -1.0, -1.0, -1.0);

    g_camTestResults.printSummary();
}

// 主函数：设置控制台编码，配置测试参数并运行
int main() {
#if defined(_WIN32)
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    // 统一调用拆分后的测试用例
    runAllCameraTests();
    return g_camTestResults.failedTests > 0 ? 1 : 0;
}


