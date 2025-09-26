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

// 配置相机参数（静默模式，减少输出）
static bool configureCameraParams(void* handle, const CameraParams& params) {
    // 1. 配置曝光参数
    if (params.exposureTimeUs > 0) {
        int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTimeUs);
        if (nRet != MV_OK) {
            std::cerr << "设置曝光时间失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
            return false;
        }
    }
    
    // 2. 配置自动曝光模式
    if (params.exposureAutoMode) {
        MV_CC_SetEnumValue(handle, "ExposureAuto", 2); // 2 = 连续自动曝光
        } else {
        MV_CC_SetEnumValue(handle, "ExposureAuto", 0); // 0 = 关闭自动曝光
    }
    
    // 3. 配置增益参数
    if (params.gainValue > 0) {
        int nRet = MV_CC_SetFloatValue(handle, "Gain", params.gainValue);
        if (nRet != MV_OK) {
            std::cerr << "设置增益失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
            return false;
        }
    }
    
    // 4. 配置自动增益模式
    if (params.gainAutoMode) {
        MV_CC_SetEnumValue(handle, "GainAuto", 2); // 2 = 连续自动增益
        } else {
        MV_CC_SetEnumValue(handle, "GainAuto", 0); // 0 = 关闭自动增益
    }
    
    // 5. 配置帧率
    if (params.frameRate > 0) {
        MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", params.frameRate);
    }
    
    // 6. 配置触发延时
    if (params.triggerDelayUs > 0) {
        MV_CC_SetFloatValue(handle, "TriggerDelay", params.triggerDelayUs);
    }
    
    return true;
}

// 单次相机拍摄函数 - 完整的相机生命周期管理
// 在每次投影仪步进时调用，实现更稳健的拍摄流程
static bool captureSingleImage(
    const std::string& cameraSerial,
    const CameraParams& params,
    const std::string& outputPath,
    const std::string& direction = "V"
) {
    void* handle = nullptr;
    bool success = false;
    
    try {
        // 1. 枚举设备
        MV_CC_DEVICE_INFO_LIST deviceList{};
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
        if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
            std::cerr << "[" << direction << "] 未发现可用相机" << std::endl;
            return false;
        }
        
        // 2. 选择设备
        MV_CC_DEVICE_INFO* pSelectedDevice = nullptr;
        if (!cameraSerial.empty() && cameraSerial != "NULL") {
            for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
                MV_CC_DEVICE_INFO* pInfo = deviceList.pDeviceInfo[i];
                const char* serial = (pInfo->nTLayerType == MV_USB_DEVICE)
                    ? (const char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber
                    : (const char*)pInfo->SpecialInfo.stGigEInfo.chSerialNumber;
                if (serial && cameraSerial == serial) { 
                    pSelectedDevice = pInfo; 
                    break; 
                }
            }
        }
        if (!pSelectedDevice) pSelectedDevice = deviceList.pDeviceInfo[0];
        
        // 3. 创建句柄并打开设备
        nRet = MV_CC_CreateHandle(&handle, pSelectedDevice);
        if (nRet != MV_OK || !handle) {
            std::cerr << "[" << direction << "] 创建相机句柄失败" << std::endl;
            return false;
        }
        
        nRet = MV_CC_OpenDevice(handle);
        if (nRet != MV_OK) {
            std::cerr << "[" << direction << "] 打开相机失败" << std::endl;
            MV_CC_DestroyHandle(handle);
            return false;
        }
        
        // 4. 设置像素格式和配置相机参数
        MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
        
        if (!configureCameraParams(handle, params)) {
            std::cerr << "[" << direction << "] 配置相机参数失败" << std::endl;
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            return false;
        }
        
        // 5. 验证和调整相机参数
        MVCC_FLOATVALUE actualExposure, actualGain;
        MV_CC_GetFloatValue(handle, "ExposureTime", &actualExposure);
        MV_CC_GetFloatValue(handle, "Gain", &actualGain);
        
        std::cout << "[" << direction << "] 实际曝光时间: " << actualExposure.fCurValue << " μs" << std::endl;
        std::cout << "[" << direction << "] 实际增益: " << actualGain.fCurValue << std::endl;
        
        // 确保自动曝光和自动增益完全关闭
        MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
        MV_CC_SetEnumValue(handle, "GainAuto", 0);
        
        // 不进行自动增益补偿，保持用户设定的增益值
        
        // 6. 设置触发模式和开始取流
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
        if (nRet != MV_OK) {
            std::cerr << "[" << direction << "] 设置触发模式失败" << std::endl;
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            return false;
        }
        
        MV_CC_SetEnumValue(handle, "TriggerSource", 7); // 软件触发
        MV_CC_SetEnumValue(handle, "AcquisitionMode", 2); // 连续采集
        
        nRet = MV_CC_StartGrabbing(handle);
        if (nRet != MV_OK) {
            std::cerr << "[" << direction << "] 开始取流失败" << std::endl;
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            return false;
        }
        
        // 相机预热并发送软触发
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "[" << direction << "] 发送软触发命令..." << std::endl;
        
        nRet = MV_CC_SetCommandValue(handle, "TriggerSoftware");
        if (nRet != MV_OK) {
            std::cerr << "[" << direction << "] 软触发失败" << std::endl;
            MV_CC_StopGrabbing(handle);
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            return false;
        }
        
        // 等待相机完成曝光和传输
        MVCC_FLOATVALUE currentExposure;
        int waitTimeMs = 2000;
        if (MV_CC_GetFloatValue(handle, "ExposureTime", &currentExposure) == MV_OK) {
            waitTimeMs = static_cast<int>(static_cast<double>(currentExposure.fCurValue) / 1000.0) + 1000;
            if (waitTimeMs > 8000) waitTimeMs = 8000;
            if (waitTimeMs < 1000) waitTimeMs = 1000;
        }
        std::cout << "[" << direction << "] 等待相机完成曝光和传输: " << waitTimeMs << "ms" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(waitTimeMs));
        
        // 7. 获取图像（带重试机制和质量检查）
        std::cout << "[" << direction << "] 开始获取图像..." << std::endl;
        bool imageCaptured = false;
        int retryCount = 0;
        const int maxRetries = 3;
        
        while (!imageCaptured && retryCount < maxRetries) {
            if (retryCount > 0) {
                // 重试前重新配置触发模式
                MV_CC_SetEnumValue(handle, "TriggerMode", 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                MV_CC_SetEnumValue(handle, "TriggerMode", 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                MV_CC_SetCommandValue(handle, "TriggerSoftware");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            
            MV_FRAME_OUT stOutFrame = {0};
            nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 5000);
            
            if (nRet == MV_OK) {
                if (stOutFrame.stFrameInfo.nWidth > 0 && stOutFrame.stFrameInfo.nHeight > 0 && stOutFrame.pBufAddr != nullptr) {
                    std::cout << "[" << direction << "] 图像获取成功，等待数据稳定..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    
                    // 创建图像并检查质量
                    cv::Mat img(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC1, stOutFrame.pBufAddr);
                    
                    cv::Scalar meanValue = cv::mean(img);
                    cv::Scalar stdValue;
                    cv::meanStdDev(img, meanValue, stdValue);
                    double avgBrightness = meanValue[0];
                    double brightnessStd = stdValue[0];
                    std::cout << "[" << direction << "] 图像平均亮度: " << avgBrightness 
                              << ", 亮度标准差: " << brightnessStd << std::endl;
                    
                    // 动态调整图像质量检查阈值（根据实际曝光时间）
                    bool imageQualityOK = true;
                    std::string qualityIssue = "";
                    
                    // 根据实际曝光时间调整亮度阈值
                    double minBrightness = 8.0;  // 降低最小亮度要求
                    double maxBrightness = 240.0;
                    double minStdDev = 3.0;      // 降低对比度要求
                    
                    if (actualExposure.fCurValue < 10000.0f) {
                        // 如果曝光时间较短，进一步降低要求
                        minBrightness = 6.0;
                        minStdDev = 2.0;
                        std::cout << "[" << direction << "] 曝光时间较短，调整质量检查阈值" << std::endl;
                    }
                    
                    if (avgBrightness < minBrightness) {
                        imageQualityOK = false;
                        qualityIssue = "图像过暗 (亮度: " + std::to_string(avgBrightness) + ", 阈值: " + std::to_string(minBrightness) + ")";
                    } else if (avgBrightness > maxBrightness) {
                        imageQualityOK = false;
                        qualityIssue = "图像过亮 (亮度: " + std::to_string(avgBrightness) + ")";
                    } else if (brightnessStd < minStdDev) {
                        imageQualityOK = false;
                        qualityIssue = "图像缺乏对比度 (标准差: " + std::to_string(brightnessStd) + ", 阈值: " + std::to_string(minStdDev) + ")";
                    }
                    
                    if (!imageQualityOK) {
                        std::cerr << "[" << direction << "] 图像质量不合格: " << qualityIssue << "，准备重试..." << std::endl;
                        retryCount++;
                        
                        // 释放当前缓存
                        MV_CC_FreeImageBuffer(handle, &stOutFrame);
                        
                        // 如果还有重试机会，继续循环
                        if (retryCount < maxRetries) {
                            continue;
        } else {
                            std::cerr << "[" << direction << "] 达到最大重试次数，跳过保存" << std::endl;
                            break;
                        }
                    } else {
                        // 图像质量合格，再次延时确保数据完全稳定
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        
                        // 保存图像
                        if (cv::imwrite(outputPath, img)) {
                            std::cout << "[" << direction << "] 图像已保存: " << outputPath 
                                      << " (尺寸: " << stOutFrame.stFrameInfo.nWidth << "x" << stOutFrame.stFrameInfo.nHeight 
                                      << ", 亮度: " << avgBrightness 
                                      << ", 标准差: " << brightnessStd << ")" << std::endl;
                            success = true;
                            imageCaptured = true;
                        } else {
                            std::cerr << "[" << direction << "] 保存图像失败: " << outputPath << std::endl;
                            retryCount++;
                        }
                    }
                    
                    // 释放缓存
                    MV_CC_FreeImageBuffer(handle, &stOutFrame);
                } else {
                    std::cerr << "[" << direction << "] 获取到无效图像数据" << std::endl;
                    retryCount++;
                }
        } else {
                std::cerr << "[" << direction << "] 获取图像失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
                retryCount++;
                
                // 尝试获取更多调试信息
                if (retryCount >= maxRetries) {
                    MVCC_ENUMVALUE currentTriggerMode;
                    if (MV_CC_GetEnumValue(handle, "TriggerMode", &currentTriggerMode) == MV_OK) {
                        std::cout << "[" << direction << "] 当前触发模式: " << (currentTriggerMode.nCurValue == 1 ? "On" : "Off") << std::endl;
                    }
                    
                    MVCC_ENUMVALUE currentTriggerSource;
                    if (MV_CC_GetEnumValue(handle, "TriggerSource", &currentTriggerSource) == MV_OK) {
                        std::cout << "[" << direction << "] 当前触发源: " << currentTriggerSource.nCurValue << std::endl;
                    }
                }
            }
        }
        
        // 15. 停止取流
        MV_CC_StopGrabbing(handle);
        
        // 16. 关闭设备
        MV_CC_CloseDevice(handle);
        
        // 17. 销毁句柄
        MV_CC_DestroyHandle(handle);
        
    } catch (const std::exception& e) {
        std::cerr << "[" << direction << "] 相机操作异常: " << e.what() << std::endl;
        if (handle) {
            MV_CC_StopGrabbing(handle);
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
        }
    }
    
    return success;
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


// ========== 仅垂直条纹的"步进投影 + 软触发采集"（新版本：每次步进时初始化相机） ==========
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
        // 投影仪初始化
        std::cout << "\n=== 垂直条纹投影与拍摄 ===" << std::endl;
        std::cout << "连接投影仪: " << projectorModel << std::endl;
        slmaster::device::ProjectorFactory factory;
        Projector* projector = factory.getProjector(projectorModel);
        if (!projector || !projector->connect()) {
            std::cerr << "投影仪连接失败" << std::endl;
            return false;
        }
        std::cout << "投影仪已连接" << std::endl;

        // 相机参数配置
        CameraParams params;
        if (useSavedParams) { 
            loadCameraParams(params); 
        } else {
            params.exposureTimeUs = 20000.0f;
            params.gainValue = 8.0f;
            params.frameRate = 5.0f;
            params.exposureAutoMode = false; 
            params.gainAutoMode = false; 
            params.triggerDelayUs = 0;
            params.enableChunkData = false; 
            params.printCurrentParams = true;
        }
        
        // 确保参数在合理范围内
        if (params.exposureTimeUs < 5000.0f) params.exposureTimeUs = 5000.0f;
        if (params.gainValue < 3.0f) params.gainValue = 3.0f;
        
        // 显示相机参数
        std::cout << "相机参数: 曝光=" << params.exposureTimeUs << "μs, 增益=" << params.gainValue 
                  << ", 帧率=" << params.frameRate << "fps" << std::endl;

        // 创建输出目录
        std::string saveDir = outputDir.empty() ? (std::filesystem::current_path() / "images").string() : outputDir;
        try { 
            std::filesystem::create_directories(saveDir); 
        } catch (...) {
            std::cerr << "[垂直] 创建输出目录失败: " << saveDir << std::endl;
            projector->disConnect();
            return false;
        }

        // 生成垂直条纹（前N张）
        auto imgs = generatePhaseShiftFringeImages(deviceWidth, deviceHeight, frequency, intensity, offset, noiseStd, steps);
        if ((int)imgs.size() != steps * 2) { 
            std::cerr << u8"垂直条纹：生成图像失败" << std::endl; 
            projector->disConnect();
            return false;
        }
        
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

        std::cout << "装载图案表..." << std::endl;
        if (!projector->populatePatternTableData(patternSets)) {
            std::cerr << "装载图案表失败" << std::endl;
            projector->disConnect(); 
            return false;
        }

        // 投影仪稳定流程
        std::cout << "执行投影仪稳定流程..." << std::endl;
        projector->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
            projector->disConnect();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!projector->connect()) { 
            projector->disConnect(); 
            return false; 
        }
        if (!projector->populatePatternTableData(patternSets)) { 
            projector->disConnect(); 
            return false; 
        }
        
        projector->setLEDCurrent(0.9, 0.9, 0.9);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        if (!projector->project(true)) { 
            projector->disConnect(); 
            return false; 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        projector->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        if (!projector->project(true)) { 
            projector->disConnect(); 
            return false; 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // 暂停投影，准备步进控制
        std::cout << "暂停投影，准备步进控制..." << std::endl;
        projector->pause();
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        int waitMs = std::max(
            (patternSets[0].preExposureTime_ + patternSets[0].exposureTime_ + patternSets[0].postExposureTime_) / 1000 + 10,
            50
        );

        // 主循环：开始拍摄
        std::cout << "\n=== 开始垂直条纹拍摄 ===" << std::endl;
        std::cout << "步数: " << steps << ", 步进等待时间: " << waitMs << "ms" << std::endl;

        bool allSuccess = true;
        for (int i = 0; i < steps; ++i) {
            std::cout << "\n--- 第 " << (i+1) << "/" << steps << " 帧 ---" << std::endl;
            
            // 投影仪步进
            if (!projector->step()) { 
                std::cerr << "步进失败" << std::endl; 
                allSuccess = false;
                break; 
            }
            
            // 立即暂停，确保投影仪停留在当前帧（防止连续投影）
            projector->pause();
            
            // 等待投影仪稳定
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
            if (i < 2) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            // 相机拍摄 - 垂直条纹命名为 I1-IN
            std::string outputPath = (std::filesystem::path(saveDir) / ("I" + std::to_string(i+1) + ".png")).string();
            
            bool captureSuccess = false;
            int captureRetryCount = 0;
            const int maxCaptureRetries = 3;
            
            while (!captureSuccess && captureRetryCount < maxCaptureRetries) {
                if (captureRetryCount > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                
                captureSuccess = captureSingleImage(cameraSerial, params, outputPath, "V");
                captureRetryCount++;
                
                if (!captureSuccess) {
                    if (captureRetryCount >= maxCaptureRetries) {
                        std::cerr << "第 " << (i+1) << " 帧拍摄达到最大重试次数，停止流程" << std::endl;
                        allSuccess = false;
                        break;
                    }
                }
            }
            
            if (!captureSuccess) {
                std::cerr << "拍摄失败，中断流程" << std::endl;
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }

        std::cout << "\n垂直条纹拍摄完成!" << std::endl;
        projector->stop();
            projector->disConnect();
        return allSuccess;
    } catch (const std::exception& e) {
        std::cerr << "[垂直] 异常: " << e.what() << std::endl;
            return false;
    }
}

// ========== 仅水平条纹的"步进投影 + 软触发采集"（新版本：每次步进时初始化相机） ==========
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
        // 投影仪初始化
        std::cout << "\n=== 水平条纹投影与拍摄 ===" << std::endl;
        std::cout << "连接投影仪: " << projectorModel << std::endl;
        slmaster::device::ProjectorFactory factory;
        Projector* projector = factory.getProjector(projectorModel);
        if (!projector || !projector->connect()) { 
            std::cerr << "投影仪连接失败" << std::endl; 
            return false; 
        }
        std::cout << "投影仪已连接" << std::endl;

        // 相机参数配置
        CameraParams params; 
        if (useSavedParams) { 
            loadCameraParams(params); 
        } else { 
            params.exposureTimeUs = 20000.0f;
            params.gainValue = 8.0f;
            params.frameRate = 5.0f;
            params.exposureAutoMode = false; 
            params.gainAutoMode = false; 
            params.triggerDelayUs = 0; 
            params.enableChunkData = false; 
            params.printCurrentParams = true; 
        }
        
        // 确保参数在合理范围内
        if (params.exposureTimeUs < 5000.0f) params.exposureTimeUs = 5000.0f;
        if (params.gainValue < 3.0f) params.gainValue = 3.0f;
        
        // 显示相机参数
        std::cout << "相机参数: 曝光=" << params.exposureTimeUs << "μs, 增益=" << params.gainValue 
                  << ", 帧率=" << params.frameRate << "fps" << std::endl;

        // 创建输出目录
        std::string saveDir = outputDir.empty() ? (std::filesystem::current_path() / "images").string() : outputDir;
        try { 
            std::filesystem::create_directories(saveDir); 
        } catch (...) {
            std::cerr << "创建输出目录失败: " << saveDir << std::endl;
            projector->disConnect();
            return false;
        }

        // 生成水平条纹（后N张）
        auto imgs = generatePhaseShiftFringeImages(deviceWidth, deviceHeight, frequency, intensity, offset, noiseStd, steps);
        if ((int)imgs.size() != steps * 2) { 
            std::cerr << "生成图像失败" << std::endl; 
            projector->disConnect();
            return false;
        }
        
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

        std::cout << "装载图案表..." << std::endl;
        if (!projector->populatePatternTableData(patternSets)) { 
            projector->disConnect(); 
            return false; 
        }

        // 投影仪稳定流程
        std::cout << "执行投影仪稳定流程..." << std::endl;
        projector->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        projector->disConnect();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!projector->connect()) { 
            projector->disConnect(); 
            return false; 
        }
        if (!projector->populatePatternTableData(patternSets)) { 
            projector->disConnect(); 
            return false; 
        }
        
        projector->setLEDCurrent(0.9, 0.9, 0.9);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        if (!projector->project(true)) { 
            projector->disConnect(); 
            return false; 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        projector->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!projector->project(true)) { 
            projector->disConnect(); 
            return false; 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // 暂停投影，准备步进控制
        std::cout << "暂停投影，准备步进控制..." << std::endl;
        projector->pause();
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        
        int waitMs = std::max(
            (patternSets[0].preExposureTime_ + patternSets[0].exposureTime_ + patternSets[0].postExposureTime_) / 1000 + 10, 
            50
        );
        // 主循环：开始拍摄
        std::cout << "\n=== 开始水平条纹拍摄 ===" << std::endl;
        std::cout << "步数: " << steps << ", 步进等待时间: " << waitMs << "ms" << std::endl;
        
        bool allSuccess = true;
        for (int i = 0; i < steps; ++i) {
            std::cout << "\n--- 第 " << (i+1) << "/" << steps << " 帧 ---" << std::endl;
            
            // 投影仪步进
            if (!projector->step()) { 
                std::cerr << "步进失败" << std::endl; 
                allSuccess = false;
                break; 
            }
            
            // 立即暂停，确保投影仪停留在当前帧（防止连续投影）
            projector->pause();
            
            // 等待投影仪稳定
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
            if (i < 2) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            // 相机拍摄 - 水平条纹命名为 I(N+1)-I(2N)
            std::string outputPath = (std::filesystem::path(saveDir) / ("I" + std::to_string(steps + i + 1) + ".png")).string();
            
            bool captureSuccess = false;
            int captureRetryCount = 0;
            const int maxCaptureRetries = 3;
            
            while (!captureSuccess && captureRetryCount < maxCaptureRetries) {
                if (captureRetryCount > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                
                captureSuccess = captureSingleImage(cameraSerial, params, outputPath, "H");
                captureRetryCount++;
                
                if (!captureSuccess) {
                    if (captureRetryCount >= maxCaptureRetries) {
                        std::cerr << "第 " << (i+1) << " 帧拍摄达到最大重试次数，停止流程" << std::endl;
                        allSuccess = false;
                        break;
                    }
                }
            }
            
            if (!captureSuccess) {
                std::cerr << "拍摄失败，中断流程" << std::endl;
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }

        std::cout << "\n水平条纹拍摄完成!" << std::endl;
        projector->stop();
        projector->disConnect();
        return allSuccess;
    } catch (const std::exception& e) {
        std::cerr << "水平条纹异常: " << e.what() << std::endl;
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
    
    std::cout << "=== 投影仪与相机协作演示 ===" << std::endl;
    std::cout << "本程序将自动生成相移条纹图像，并实现投影仪投影与相机采集的同步协作" << std::endl;
    std::cout << "相机参数将从保存的配置文件中读取（如果存在）" << std::endl;
    std::cout << std::endl;
    
    // 示例：相机序列号与保存目录可按需填写；序列号传 "NULL" 表示自动选择第一台
    const std::string cameraSerial = "NULL"; // 或者例如 "DA1015150"
    const std::string saveDir = "images";    // 图像保存目录
    
    /*
    这里的参数分别是：projectorModel：投影仪型号，deviceWidth：投影宽度，deviceHeight：投影高度，steps：相移步数，frequency：条纹频率，intensity：条纹强度，
    offset：亮度偏移，noiseStd：噪声标准差，cameraSerial：相机序列号，saveDir：图像保存目录，useSavedParams：是否使用保存的相机参数
    */
    bool successV = slmaster_demo::runVerticalProjectStepAndCapture(
        "DLP4710", 1920, 1080, 4, 15, 100, 128, 0.0, cameraSerial, saveDir, true);

    /*
    这里的参数分别是：projectorModel：投影仪型号，deviceWidth：投影宽度，deviceHeight：投影高度，steps：相移步数，frequency：条纹频率，intensity：条纹强度，
    offset：亮度偏移，noiseStd：噪声标准差，cameraSerial：相机序列号，saveDir：图像保存目录，useSavedParams：是否使用保存的相机参数
    */
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


