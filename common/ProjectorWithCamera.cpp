// 投影仪与相机协作演示（完整实现版本）
//
// 设计目标：
// - 提供一个可直接复用的"步进投影 + 软触发采集"的协作流程样例；
// - 相机与投影仪均采用"软触发"：由软件下发指令来推进投影以及触发相机采集；
// - 自动生成 N 步相移条纹，顺序：垂直 N 张 + 水平 N 张，总计 2N 张；
// - 每投影一张（step一次），便触发一次相机采集并保存图像；
// - 支持从CameraTest.cpp保存的参数文件读取相机配置；
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
#include <algorithm>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

// ========== MVS 工业相机 SDK 头文件 ==========
#include "MvCameraControl.h"
// 图像保存
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace slmaster_demo {

// ========== 相机参数配置结构体（与CameraTest.cpp保持一致） ==========
struct CameraParams {
    // 曝光参数
    float exposureTimeUs = -1.0f;        // 曝光时间（微秒），-1表示使用默认值
    bool exposureAutoMode = false;       // 是否启用自动曝光
    
    // 增益参数
    float gainValue = -1.0f;             // 增益值，-1表示使用默认值
    bool gainAutoMode = false;           // 是否启用自动增益
    
    // 帧率参数
    float frameRate = -1.0f;             // 帧率（fps），-1表示使用默认值
    
    // 触发参数
    int triggerDelayUs = 0;              // 触发延时（微秒）
    
    // 其他参数
    bool enableChunkData = false;        // 是否启用块数据
    bool printCurrentParams = true;      // 是否打印当前参数
    
    // 参数范围信息（用于验证）
    struct {
        float exposureMin = 0.0f, exposureMax = 0.0f;
        float gainMin = 0.0f, gainMax = 0.0f;
        float frameRateMin = 0.0f, frameRateMax = 0.0f;
    } ranges;
};

// 参数文件管理（与CameraTest.cpp保持一致）
const std::string PARAMS_FILE = "camera_params.txt";

// 从文件读取相机参数（与CameraTest.cpp保持一致）
static bool loadCameraParams(CameraParams& params, const std::string& filename = PARAMS_FILE) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "参数文件不存在: " << filename << "，将使用默认参数" << std::endl;
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            // 跳过空行和注释行
            if (line.empty() || line[0] == '#') {
                continue;
            }
            
            // 解析键值对
            size_t pos = line.find('=');
            if (pos == std::string::npos) {
                continue;
            }
            
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);
            
            // 去除前后空格
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            // 设置参数值
            if (key == "exposureTimeUs") {
                params.exposureTimeUs = std::stof(value);
            } else if (key == "exposureAutoMode") {
                params.exposureAutoMode = (value == "1");
            } else if (key == "gainValue") {
                params.gainValue = std::stof(value);
            } else if (key == "gainAutoMode") {
                params.gainAutoMode = (value == "1");
            } else if (key == "frameRate") {
                params.frameRate = std::stof(value);
            } else if (key == "triggerDelayUs") {
                params.triggerDelayUs = std::stoi(value);
            } else if (key == "enableChunkData") {
                params.enableChunkData = (value == "1");
            } else if (key == "printCurrentParams") {
                params.printCurrentParams = (value == "1");
            }
        }
        
        file.close();
        std::cout << "参数已从文件加载: " << filename << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "读取参数文件失败: " << e.what() << std::endl;
        return false;
    }
}

// 配置相机参数（与CameraTest.cpp保持一致）
static bool configureCameraParams(void* handle, const CameraParams& params) {
    std::cout << "\n=== 配置相机参数 ===" << std::endl;
    
    // 1. 配置曝光参数
    if (params.exposureTimeUs > 0) {
        std::cout << "设置曝光时间: " << params.exposureTimeUs << " μs" << std::endl;
        int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTimeUs);
        if (nRet != MV_OK) {
            std::cerr << "设置曝光时间失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
            return false;
        }
        std::cout << "曝光时间设置成功" << std::endl;
    }
    
    // 2. 配置自动曝光模式
    if (params.exposureAutoMode) {
        std::cout << "启用自动曝光模式" << std::endl;
        int nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 2); // 2 = 连续自动曝光
        if (nRet != MV_OK) {
            std::cout << "启用自动曝光失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        } else {
            std::cout << "自动曝光模式启用成功" << std::endl;
        }
    } else {
        // 关闭自动曝光
        int nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 0); // 0 = 关闭自动曝光
        if (nRet == MV_OK) {
            std::cout << "自动曝光模式已关闭" << std::endl;
        }
    }
    
    // 3. 配置增益参数
    if (params.gainValue > 0) {
        std::cout << "设置增益: " << params.gainValue << std::endl;
        int nRet = MV_CC_SetFloatValue(handle, "Gain", params.gainValue);
        if (nRet != MV_OK) {
            std::cerr << "设置增益失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
            return false;
        }
        std::cout << "增益设置成功" << std::endl;
    }
    
    // 4. 配置自动增益模式
    if (params.gainAutoMode) {
        std::cout << "启用自动增益模式" << std::endl;
        int nRet = MV_CC_SetEnumValue(handle, "GainAuto", 2); // 2 = 连续自动增益
        if (nRet != MV_OK) {
            std::cout << "启用自动增益失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        } else {
            std::cout << "自动增益模式启用成功" << std::endl;
        }
    } else {
        // 关闭自动增益
        int nRet = MV_CC_SetEnumValue(handle, "GainAuto", 0); // 0 = 关闭自动增益
        if (nRet == MV_OK) {
            std::cout << "自动增益模式已关闭" << std::endl;
        }
    }
    
    // 5. 配置帧率
    if (params.frameRate > 0) {
        std::cout << "设置帧率: " << params.frameRate << " fps" << std::endl;
        int nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", params.frameRate);
        if (nRet != MV_OK) {
            std::cout << "设置帧率失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        } else {
            std::cout << "帧率设置成功" << std::endl;
        }
    }
    
    // 6. 配置触发延时
    if (params.triggerDelayUs > 0) {
        std::cout << "设置触发延时: " << params.triggerDelayUs << " μs" << std::endl;
        int nRet = MV_CC_SetFloatValue(handle, "TriggerDelay", params.triggerDelayUs);
        if (nRet != MV_OK) {
            std::cout << "设置触发延时失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        } else {
            std::cout << "触发延时设置成功" << std::endl;
        }
    }
    
    std::cout << "相机参数配置完成" << std::endl;
    return true;
}

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


// ========== 新增：仅垂直条纹的“步进投影 + 软触发采集” ==========
bool runVerticalProjectStepAndCapture(
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
    bool useSavedParams
) {
    using namespace slmaster::device;
    try {
        // 投影仪
        slmaster::device::ProjectorFactory factory;
        Projector* projector = factory.getProjector(projectorModel);
        if (!projector || !projector->connect()) {
            std::cerr << u8"垂直条纹：投影仪连接失败" << std::endl;
            return false;
        }

        // 相机初始化
        MV_CC_DEVICE_INFO_LIST deviceList{};
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
        if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
            std::cerr << u8"垂直条纹：未发现可用相机" << std::endl;
            projector->disConnect();
            return false;
        }
        MV_CC_DEVICE_INFO* pSelectedDevice = nullptr;
        if (!cameraSerial.empty() && cameraSerial != "NULL") {
            for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
                MV_CC_DEVICE_INFO* pInfo = deviceList.pDeviceInfo[i];
                const char* serial = (pInfo->nTLayerType == MV_USB_DEVICE)
                    ? (const char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber
                    : (const char*)pInfo->SpecialInfo.stGigEInfo.chSerialNumber;
                if (serial && cameraSerial == serial) { pSelectedDevice = pInfo; break; }
            }
        }
        if (!pSelectedDevice) pSelectedDevice = deviceList.pDeviceInfo[0];
        void* cameraHandle = nullptr;
        nRet = MV_CC_CreateHandle(&cameraHandle, pSelectedDevice);
        if (nRet != MV_OK || !cameraHandle) {
            std::cerr << u8"垂直条纹：创建相机句柄失败" << std::endl;
            projector->disConnect();
            return false;
        }
        if (MV_CC_OpenDevice(cameraHandle) != MV_OK) {
            std::cerr << u8"垂直条纹：打开相机失败" << std::endl;
            MV_CC_DestroyHandle(cameraHandle);
            projector->disConnect();
            return false;
        }
        // 基础触发配置
        MV_CC_SetEnumValueByString(cameraHandle, "PixelFormat", "Mono8");
        MV_CC_SetEnumValueByString(cameraHandle, "TriggerSelector", "FrameStart");
        MV_CC_SetEnumValue(cameraHandle, "TriggerMode", 1);
        MV_CC_SetEnumValueByString(cameraHandle, "TriggerSource", "Software");
        MV_CC_SetEnumValueByString(cameraHandle, "AcquisitionMode", "Continuous");

        // 相机参数
        CameraParams params;
        if (useSavedParams) { loadCameraParams(params); }
        else {
            params.exposureTimeUs = 10000.0f; params.gainValue = 5.0f; params.frameRate = 10.0f;
            params.exposureAutoMode = false; params.gainAutoMode = false; params.triggerDelayUs = 0;
            params.enableChunkData = false; params.printCurrentParams = true;
        }
        configureCameraParams(cameraHandle, params);

        // 回调上下文
        struct CbCtx { std::atomic<int> idx{0}; int total{0}; std::string dir; } ctx;
        ctx.total = steps; ctx.dir = outputDir.empty() ? (std::filesystem::current_path() / "images").string() : outputDir;
        try { std::filesystem::create_directories(ctx.dir); } catch (...) {}
        auto ImageCallbackEx = [](unsigned char* pData, MV_FRAME_OUT_INFO_EX* info, void* p) {
            if (!pData || !info || !p) return;
            CbCtx* c = reinterpret_cast<CbCtx*>(p);
            int idx = 1 + c->idx.fetch_add(1);
            if (idx <= c->total) {
                cv::Mat img(info->nHeight, info->nWidth, CV_8UC1, pData);
                cv::imwrite((std::filesystem::path(c->dir) / ("I" + std::to_string(idx) + "_V.png")).string(), img);
            }
        };
        MV_CC_RegisterImageCallBackEx(cameraHandle, ImageCallbackEx, &ctx);
        if (MV_CC_StartGrabbing(cameraHandle) != MV_OK) {
            std::cerr << u8"垂直条纹：开始采集失败" << std::endl;
            MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect();
            return false;
        }

        // 生成垂直条纹（前N张）
        auto imgs = generatePhaseShiftFringeImages(deviceWidth, deviceHeight, frequency, intensity, offset, noiseStd, steps);
        if ((int)imgs.size() != steps * 2) { std::cerr << u8"垂直条纹：生成图像失败" << std::endl; }
        std::vector<PatternOrderSet> patternSets(1);
        patternSets[0].exposureTime_ = 4000;
        patternSets[0].preExposureTime_ = 3000;
        patternSets[0].postExposureTime_ = 3000;
        patternSets[0].illumination_ = Blue;
        patternSets[0].invertPatterns_ = false;
        patternSets[0].isVertical_ = true;
        patternSets[0].isOneBit_ = false;
        patternSets[0].patternArrayCounts_ = deviceWidth;
        patternSets[0].imgs_.assign(imgs.begin(), imgs.begin() + steps);

        if (!projector->populatePatternTableData(patternSets)) {
            std::cerr << u8"垂直条纹：装载图案表失败" << std::endl;
            MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle);
            projector->disConnect(); return false;
        }

        // 按照 ProjectorTest 的稳定流程：停止-断开-重连-重新加载-设LED-开始投影-稳定-停止-再次开始
        projector->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        projector->disConnect();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!projector->connect()) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); return false; }
        if (!projector->populatePatternTableData(patternSets)) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }
        projector->setLEDCurrent(0.9, 0.9, 0.9);
        if (!projector->project(true)) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        projector->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!projector->project(true)) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        int waitMs = std::max(
            (patternSets[0].preExposureTime_ + patternSets[0].exposureTime_ + patternSets[0].postExposureTime_) / 1000 + 10,
            50
        );

        for (int i = 0; i < steps; ++i) {
            if (!projector->step()) { std::cerr << u8"垂直条纹：步进失败" << std::endl; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));

            // 触发相机
            std::string triggerCommand = "TriggerSoftware";
            MVCC_ENUMVALUE selector; if (MV_CC_GetEnumValue(cameraHandle, "TriggerSelector", &selector) == MV_OK && selector.nCurValue == 0) triggerCommand = "FrameTriggerSoftware";
            MV_CC_SetCommandValue(cameraHandle, triggerCommand.c_str());

            // 等待相机曝光完成
            MVCC_FLOATVALUE curExp; int camWait = 1000; if (MV_CC_GetFloatValue(cameraHandle, "ExposureTime", &curExp) == MV_OK) { camWait = (int)(curExp.fCurValue / 1000.0) + 500; if (camWait > 5000) camWait = 5000; }
            std::this_thread::sleep_for(std::chrono::milliseconds(camWait));
        }

        projector->stop();
        MV_CC_StopGrabbing(cameraHandle);
        MV_CC_CloseDevice(cameraHandle);
        MV_CC_DestroyHandle(cameraHandle);
        projector->disConnect();
        return true;
    } catch (...) {
        return false;
    }
}

// ========== 新增：仅水平条纹的“步进投影 + 软触发采集” ==========
bool runHorizontalProjectStepAndCapture(
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
    bool useSavedParams
) {
    using namespace slmaster::device;
    try {
        slmaster::device::ProjectorFactory factory;
        Projector* projector = factory.getProjector(projectorModel);
        if (!projector || !projector->connect()) { std::cerr << u8"水平条纹：投影仪连接失败" << std::endl; return false; }

        // 相机初始化与配置（同上）
        MV_CC_DEVICE_INFO_LIST deviceList{};
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
        if (nRet != MV_OK || deviceList.nDeviceNum == 0) { projector->disConnect(); return false; }
        MV_CC_DEVICE_INFO* pSelectedDevice = deviceList.pDeviceInfo[0];
        if (!cameraSerial.empty() && cameraSerial != "NULL") {
            for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
                MV_CC_DEVICE_INFO* pInfo = deviceList.pDeviceInfo[i];
                const char* serial = (pInfo->nTLayerType == MV_USB_DEVICE)
                    ? (const char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber
                    : (const char*)pInfo->SpecialInfo.stGigEInfo.chSerialNumber;
                if (serial && cameraSerial == serial) { pSelectedDevice = pInfo; break; }
            }
        }
        void* cameraHandle = nullptr;
        if (MV_CC_CreateHandle(&cameraHandle, pSelectedDevice) != MV_OK || !cameraHandle) { projector->disConnect(); return false; }
        if (MV_CC_OpenDevice(cameraHandle) != MV_OK) { MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }
        MV_CC_SetEnumValueByString(cameraHandle, "PixelFormat", "Mono8");
        MV_CC_SetEnumValueByString(cameraHandle, "TriggerSelector", "FrameStart");
        MV_CC_SetEnumValue(cameraHandle, "TriggerMode", 1);
        MV_CC_SetEnumValueByString(cameraHandle, "TriggerSource", "Software");
        MV_CC_SetEnumValueByString(cameraHandle, "AcquisitionMode", "Continuous");

        CameraParams params; if (useSavedParams) loadCameraParams(params); else { params.exposureTimeUs = 10000.0f; params.gainValue = 5.0f; params.frameRate = 10.0f; params.exposureAutoMode=false; params.gainAutoMode=false; params.triggerDelayUs=0; params.enableChunkData=false; params.printCurrentParams=true; }
        configureCameraParams(cameraHandle, params);

        struct CbCtx { std::atomic<int> idx{0}; int total{0}; std::string dir; } ctx;
        ctx.total = steps; ctx.dir = outputDir.empty() ? (std::filesystem::current_path() / "images").string() : outputDir;
        try { std::filesystem::create_directories(ctx.dir); } catch (...) {}
        auto ImageCallbackEx = [](unsigned char* pData, MV_FRAME_OUT_INFO_EX* info, void* p) {
            if (!pData || !info || !p) return; CbCtx* c = reinterpret_cast<CbCtx*>(p);
            int idx = 1 + c->idx.fetch_add(1); if (idx <= c->total) { cv::Mat img(info->nHeight, info->nWidth, CV_8UC1, pData); cv::imwrite((std::filesystem::path(c->dir) / ("I" + std::to_string(idx) + "_H.png")).string(), img); }
        };
        MV_CC_RegisterImageCallBackEx(cameraHandle, ImageCallbackEx, &ctx);
        if (MV_CC_StartGrabbing(cameraHandle) != MV_OK) { MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }

        auto imgs = generatePhaseShiftFringeImages(deviceWidth, deviceHeight, frequency, intensity, offset, noiseStd, steps);
        if ((int)imgs.size() != steps * 2) { std::cerr << u8"水平条纹：生成图像失败" << std::endl; }
        std::vector<PatternOrderSet> patternSets(1);
        patternSets[0].exposureTime_ = 4000;
        patternSets[0].preExposureTime_ = 3000;
        patternSets[0].postExposureTime_ = 3000;
        patternSets[0].illumination_ = Blue;
        patternSets[0].invertPatterns_ = false;
        patternSets[0].isVertical_ = false;
        patternSets[0].isOneBit_ = false;
        patternSets[0].patternArrayCounts_ = deviceWidth;
        patternSets[0].imgs_.assign(imgs.begin() + steps, imgs.end());

        if (!projector->populatePatternTableData(patternSets)) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }

        // 按照 ProjectorTest 的稳定流程
        projector->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        projector->disConnect();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!projector->connect()) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); return false; }
        if (!projector->populatePatternTableData(patternSets)) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }
        projector->setLEDCurrent(0.9, 0.9, 0.9);
        if (!projector->project(true)) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        projector->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!projector->project(true)) { MV_CC_StopGrabbing(cameraHandle); MV_CC_CloseDevice(cameraHandle); MV_CC_DestroyHandle(cameraHandle); projector->disConnect(); return false; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        int waitMs = std::max((patternSets[0].preExposureTime_ + patternSets[0].exposureTime_ + patternSets[0].postExposureTime_) / 1000 + 10, 50);

        for (int i = 0; i < steps; ++i) {
            if (!projector->step()) { std::cerr << u8"水平条纹：步进失败" << std::endl; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
            std::string triggerCommand = "TriggerSoftware"; MVCC_ENUMVALUE selector; if (MV_CC_GetEnumValue(cameraHandle, "TriggerSelector", &selector) == MV_OK && selector.nCurValue == 0) triggerCommand = "FrameTriggerSoftware";
            MV_CC_SetCommandValue(cameraHandle, triggerCommand.c_str());
            MVCC_FLOATVALUE curExp; int camWait = 1000; if (MV_CC_GetFloatValue(cameraHandle, "ExposureTime", &curExp) == MV_OK) { camWait = (int)(curExp.fCurValue / 1000.0) + 500; if (camWait > 5000) camWait = 5000; }
            std::this_thread::sleep_for(std::chrono::milliseconds(camWait));
        }

        projector->stop();
        MV_CC_StopGrabbing(cameraHandle);
        MV_CC_CloseDevice(cameraHandle);
        MV_CC_DestroyHandle(cameraHandle);
        projector->disConnect();
        return true;
    } catch (...) { return false; }
}

} // namespace slmaster_demo

// 示例调用（可由外部单元测试或GUI事件触发）
int main() {
#if defined(_WIN32)
    // 将控制台输入/输出代码页切换为 UTF-8，避免中文乱码
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    
    std::cout << "=== 投影仪与相机协作演示 ===" << std::endl;
    std::cout << "本程序将自动生成相移条纹图像，并实现投影仪投影与相机采集的同步协作" << std::endl;
    std::cout << "相机参数将从保存的配置文件中读取（如果存在）" << std::endl;
    std::cout << std::endl;
    
    // 示例：相机序列号与保存目录可按需填写；序列号传 "NULL" 表示自动选择第一台
    const std::string cameraSerial = "NULL"; // 或者例如 "DA1015150"
    const std::string saveDir = "images";    // 图像保存目录
    
    bool successV = slmaster_demo::runVerticalProjectStepAndCapture(
        "DLP4710", 1920, 1080, 4, 15, 100, 128, 0.0, cameraSerial, saveDir, true);
    bool successH = slmaster_demo::runHorizontalProjectStepAndCapture(
        "DLP4710", 1920, 1080, 4, 15, 100, 128, 0.0, cameraSerial, saveDir, true);

    if (successV && successH) {
        std::cout << u8"投影仪与相机协作演示完成！" << std::endl;
        std::cout << u8"图像已保存到 " << saveDir << u8" 目录" << std::endl;
    } else {
        std::cerr << u8"投影仪与相机协作演示失败！" << std::endl;
    }

    return (successV && successH) ? 0 : 1;
}


