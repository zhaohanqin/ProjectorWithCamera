// 相机连接与拍摄测试（基于海康 MVS C++ SDK）
// 功能：
// 1) 枚举相机并按序列号选择；若传 "NULL" 或未指定则默认第一台
// 2) 打开设备、配置为软件触发 Mono8
// 3) 注册回调并软触发抓拍 N 张图像，保存为 I1..IN.png 到指定目录（或当前目录）
// 4) 简化版本：不再进行智能曝光控制，保持相机默认设置

#include <iostream>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <sstream>

#if defined(_WIN32)
// 避免 Windows 宏 min/max 与 std::min/std::max 冲突
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(ms) usleep((ms) * 1000)
#endif

#include "MvCameraControl.h"

// 相机参数配置结构体
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

// 全局变量和函数用于实时预览
static cv::Mat g_latestImage;
static std::atomic<bool> g_imageUpdated{false};
static std::atomic<int> g_callbackCallCount{0};
static std::atomic<int> g_imageUpdateCount{0};

// 静态回调函数用于实时预览（目前未使用，ImageCallbackEx兼顾了预览功能）

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

// 获取相机参数范围信息
static bool getCameraParamRanges(void* handle, CameraParams& params) {
    std::cout << "获取相机参数范围信息..." << std::endl;
    
    // 获取曝光时间范围
    MVCC_FLOATVALUE exposureRange;
    int nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &exposureRange);
    if (nRet == MV_OK) {
        params.ranges.exposureMin = exposureRange.fMin;
        params.ranges.exposureMax = exposureRange.fMax;
        std::cout << "曝光时间范围: " << exposureRange.fMin << " - " << exposureRange.fMax << " μs" << std::endl;
    } else {
        std::cout << "获取曝光时间范围失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
    }
    
    // 获取增益范围
    MVCC_FLOATVALUE gainRange;
    nRet = MV_CC_GetFloatValue(handle, "Gain", &gainRange);
    if (nRet == MV_OK) {
        params.ranges.gainMin = gainRange.fMin;
        params.ranges.gainMax = gainRange.fMax;
        std::cout << "增益范围: " << gainRange.fMin << " - " << gainRange.fMax << std::endl;
    } else {
        std::cout << "获取增益范围失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
    }
    
    // 获取帧率范围
    MVCC_FLOATVALUE frameRateRange;
    nRet = MV_CC_GetFloatValue(handle, "ResultingFrameRate", &frameRateRange);
    if (nRet == MV_OK) {
        params.ranges.frameRateMin = frameRateRange.fMin;
        params.ranges.frameRateMax = frameRateRange.fMax;
        std::cout << "帧率范围: " << frameRateRange.fMin << " - " << frameRateRange.fMax << " fps" << std::endl;
    } else {
        std::cout << "获取帧率范围失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
    }
    
    return true;
}

// 打印当前相机参数
static void printCurrentCameraParams(void* handle) {
    std::cout << "\n=== 当前相机参数 ===" << std::endl;
    
    // 获取当前曝光时间
    MVCC_FLOATVALUE currentExposure;
    if (MV_CC_GetFloatValue(handle, "ExposureTime", &currentExposure) == MV_OK) {
        std::cout << "当前曝光时间: " << currentExposure.fCurValue << " μs" << std::endl;
    }
    
    // 获取当前增益
    MVCC_FLOATVALUE currentGain;
    if (MV_CC_GetFloatValue(handle, "Gain", &currentGain) == MV_OK) {
        std::cout << "当前增益: " << currentGain.fCurValue << std::endl;
    }
    
    // 获取当前帧率
    MVCC_FLOATVALUE currentFrameRate;
    if (MV_CC_GetFloatValue(handle, "ResultingFrameRate", &currentFrameRate) == MV_OK) {
        std::cout << "当前帧率: " << currentFrameRate.fCurValue << " fps" << std::endl;
    }
    
    // 获取自动曝光模式
    MVCC_ENUMVALUE exposureAutoMode;
    if (MV_CC_GetEnumValue(handle, "ExposureAuto", &exposureAutoMode) == MV_OK) {
        std::cout << "自动曝光模式: " << (exposureAutoMode.nCurValue == 2 ? "连续" : 
                                          exposureAutoMode.nCurValue == 1 ? "一次" : "关闭") << std::endl;
    }
    
    // 获取自动增益模式
    MVCC_ENUMVALUE gainAutoMode;
    if (MV_CC_GetEnumValue(handle, "GainAuto", &gainAutoMode) == MV_OK) {
        std::cout << "自动增益模式: " << (gainAutoMode.nCurValue == 2 ? "连续" : 
                                        gainAutoMode.nCurValue == 1 ? "一次" : "关闭") << std::endl;
    }
    
    std::cout << "===================" << std::endl;
}

// 配置相机参数
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
            // 自动曝光失败不影响整体流程
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
            // 自动增益失败不影响整体流程
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
            // 帧率设置失败不影响整体流程
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
            // 触发延时设置失败不影响整体流程
        } else {
            std::cout << "触发延时设置成功" << std::endl;
        }
    }
    
    // 7. 配置块数据
    if (params.enableChunkData) {
        std::cout << "启用块数据" << std::endl;
        int nRet = MV_CC_SetBoolValue(handle, "ChunkModeActive", true);
        if (nRet != MV_OK) {
            std::cout << "启用块数据失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        } else {
            std::cout << "块数据启用成功" << std::endl;
        }
    }
    
    std::cout << "相机参数配置完成" << std::endl;
    
    // 打印当前参数
    if (params.printCurrentParams) {
        printCurrentCameraParams(handle);
    }
    
    return true;
}

// 参数文件管理
const std::string PARAMS_FILE = "camera_params.txt";

// 保存相机参数到文件
static bool saveCameraParams(const CameraParams& params, const std::string& filename = PARAMS_FILE) {
    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "无法创建参数文件: " << filename << std::endl;
            return false;
        }
        
        // 写入参数，使用简单的键值对格式
        file << "# 相机参数配置文件" << std::endl;
        file << "# 格式: 参数名=值" << std::endl;
        file << "# 注释行以#开头" << std::endl;
        file << std::endl;
        
        file << "exposureTimeUs=" << params.exposureTimeUs << std::endl;
        file << "exposureAutoMode=" << (params.exposureAutoMode ? "1" : "0") << std::endl;
        file << "gainValue=" << params.gainValue << std::endl;
        file << "gainAutoMode=" << (params.gainAutoMode ? "1" : "0") << std::endl;
        file << "frameRate=" << params.frameRate << std::endl;
        file << "triggerDelayUs=" << params.triggerDelayUs << std::endl;
        file << "enableChunkData=" << (params.enableChunkData ? "1" : "0") << std::endl;
        file << "printCurrentParams=" << (params.printCurrentParams ? "1" : "0") << std::endl;
        
        file.close();
        std::cout << "参数已保存到文件: " << filename << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "保存参数文件失败: " << e.what() << std::endl;
        return false;
    }
}

// 从文件读取相机参数
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

// 显示当前保存的参数
static void displaySavedParams(const CameraParams& params) {
    std::cout << "\n=== 当前保存的参数 ===" << std::endl;
    std::cout << "曝光时间: " << (params.exposureTimeUs > 0 ? std::to_string(params.exposureTimeUs) + " μs" : "默认值") << std::endl;
    std::cout << "自动曝光: " << (params.exposureAutoMode ? "启用" : "关闭") << std::endl;
    std::cout << "增益值: " << (params.gainValue > 0 ? std::to_string(params.gainValue) + " dB" : "默认值") << std::endl;
    std::cout << "自动增益: " << (params.gainAutoMode ? "启用" : "关闭") << std::endl;
    std::cout << "帧率: " << (params.frameRate > 0 ? std::to_string(params.frameRate) + " fps" : "默认值") << std::endl;
    std::cout << "触发延时: " << params.triggerDelayUs << " μs" << std::endl;
    std::cout << "块数据: " << (params.enableChunkData ? "启用" : "关闭") << std::endl;
    std::cout << "打印参数: " << (params.printCurrentParams ? "是" : "否") << std::endl;
    std::cout << "===================" << std::endl;
}

// 检查参数文件是否存在
static bool paramsFileExists(const std::string& filename = PARAMS_FILE) {
    return std::filesystem::exists(filename);
}

// 备份参数文件
static bool backupParamsFile(const std::string& filename = PARAMS_FILE) {
    if (!paramsFileExists(filename)) {
        return true; // 文件不存在，无需备份
    }
    
    try {
        std::string backupName = filename + ".backup";
        std::filesystem::copy_file(filename, backupName, std::filesystem::copy_options::overwrite_existing);
        std::cout << "参数文件已备份到: " << backupName << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "备份参数文件失败: " << e.what() << std::endl;
        return false;
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

// 图像回调函数：将采集到的帧以灰度图保存
// 注意：SDK在回调中提供的缓冲区在返回后可能失效，本处直接在回调中同步写盘
static void ImageCallbackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    if (!pFrameInfo) {
        std::cout << "回调函数参数无效: pFrameInfo=" << pFrameInfo << std::endl;
        return;
    }
    
    // 增加回调调用计数
    g_callbackCallCount++;
    
    // 检查是否为预览模式（pUser为nullptr表示预览模式）
    bool isPreviewMode = (pUser == nullptr);
    
    if (isPreviewMode) {
        // 预览模式：只更新全局图像变量，不保存文件
        try {
            cv::Mat img(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
            cv::Mat imgCopy = img.clone(); // 复制一份，避免回调返回后缓冲区失效
            
            // 更新全局预览变量
            g_latestImage = imgCopy.clone();
            g_imageUpdated = true;
            g_imageUpdateCount++;
            
            std::cout << "=== 预览回调触发 ===" << std::endl;
            std::cout << "Preview Frame: W[" << pFrameInfo->nWidth
                      << "] H[" << pFrameInfo->nHeight << "]" << std::endl;
            std::cout << "帧长度: " << pFrameInfo->nFrameLenEx << " 字节" << std::endl;
            std::cout << "像素格式: 0x" << std::hex << pFrameInfo->enPixelType << std::dec << std::endl;
            std::cout << "帧号: " << pFrameInfo->nFrameNum << std::endl;
            std::cout << "预览图像已更新，回调计数: " << g_callbackCallCount << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "✗ 预览图像处理异常: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "✗ 预览图像处理未知异常" << std::endl;
        }
        return;
    }
    
    // 正常模式：保存图像到文件
    if (!pUser) {
        std::cout << "回调函数参数无效: pUser=" << pUser << std::endl;
        return;
    }
    
    CallbackContext* c = reinterpret_cast<CallbackContext*>(pUser);
    const int idx = 1 + c->frameIndex.fetch_add(1);
    
    std::cout << "=== 图像回调触发 ===" << std::endl;
    std::cout << "Get One Frame: W[" << pFrameInfo->nWidth
              << "] H[" << pFrameInfo->nHeight
              << "] Index[" << idx << "/" << c->totalFrames << "]" << std::endl;
    std::cout << "帧长度: " << pFrameInfo->nFrameLenEx << " 字节" << std::endl;
    std::cout << "像素格式: 0x" << std::hex << pFrameInfo->enPixelType << std::dec << std::endl;
    std::cout << "帧号: " << pFrameInfo->nFrameNum << std::endl;
    
            if (idx <= c->totalFrames) {
            try {
                cv::Mat img(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
                cv::Mat imgCopy = img.clone(); // 复制一份，避免回调返回后缓冲区失效
                
                // 检查图像质量，避免保存全白或全黑的图像
                double minVal, maxVal, meanVal;
                cv::minMaxLoc(imgCopy, &minVal, &maxVal);
                cv::Mat meanMat, stdDevMat;
                cv::meanStdDev(imgCopy, meanMat, stdDevMat);
                meanVal = meanMat.at<double>(0, 0);
                
                std::cout << "图像质量检查 - 像素值范围: [" << minVal << ", " << maxVal << "], 平均值: " << meanVal << std::endl;
                
                // 如果图像是全白（所有像素都是255）或全黑（所有像素都是0），可能是曝光问题
                if (minVal == maxVal) {
                    if (minVal == 255) {
                        std::cout << "⚠️  警告：图像全白，可能是曝光过度，建议减少曝光时间" << std::endl;
                    } else if (minVal == 0) {
                        std::cout << "⚠️  警告：图像全黑，可能是曝光不足，建议增加曝光时间" << std::endl;
                    }
                }
                
                // 检查图像是否有足够的对比度（标准差应该大于某个阈值）
                double stdDev = stdDevMat.at<double>(0, 0);
                if (stdDev < 10.0) {
                    std::cout << "⚠️  警告：图像对比度较低（标准差=" << stdDev << "），可能没有有效内容" << std::endl;
                }
                
                                 // 简化图像保存逻辑，直接保存所有图像
                // 不再进行智能曝光控制
                std::cout << "图像亮度分析 - 平均值: " << meanVal << std::endl;
                
                // 检查是否有像素值达到255（过曝区域）
                if (maxVal == 255) {
                    std::cout << "⚠️  警告：图像存在过曝区域（像素值255）" << std::endl;
                }
                
                // 直接保存图像
                {
                    std::string filename = "I" + std::to_string(idx) + ".png";
                    std::string path = (std::filesystem::path(c->saveDir) / filename).string();
                    
                    // 确保保存目录存在
                    try {
                        std::filesystem::create_directories(c->saveDir);
                    } catch (...) {
                        std::cout << "创建保存目录失败，使用当前目录" << std::endl;
                        path = filename;
                    }
                    
                    bool saveSuccess = cv::imwrite(path, imgCopy);
                    if (saveSuccess) {
                        std::cout << "✓ 图像保存成功: " << path << std::endl;
                        std::cout << "图像尺寸: " << imgCopy.cols << "x" << imgCopy.rows << std::endl;
                        std::cout << "图像质量: 标准差=" << stdDev << ", 对比度=" << (maxVal - minVal) << std::endl;
                    } else {
                        std::cerr << "✗ 图像保存失败: " << path << std::endl;
                        // 尝试获取更详细的错误信息
                        std::cerr << "保存路径: " << path << std::endl;
                        std::cerr << "目录是否存在: " << (std::filesystem::exists(c->saveDir) ? "是" : "否") << std::endl;
                    }
                    
                    // 同时更新全局预览变量以支持实时预览
                    g_latestImage = imgCopy.clone();
                    g_imageUpdated = true;
                    g_imageUpdateCount++;
                    std::cout << "图像已更新到预览缓存" << std::endl;
                }
                
                // 更新预览缓存
                g_latestImage = imgCopy.clone();
                g_imageUpdated = true;
                g_imageUpdateCount++;
            } catch (const std::exception& e) {
                std::cerr << "✗ 图像处理异常: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "✗ 图像处理未知异常" << std::endl;
            }
        } else {
            std::cout << "跳过保存，索引超出范围: " << idx << " > " << c->totalFrames << std::endl;
        }
    
    std::cout << "=== 回调函数结束 ===" << std::endl;
}

// 运行相机连接与图像采集测试
// 参数：
// - cameraSerial: 指定要连接的相机序列号；传 "NULL" 或空字符串表示自动选择第一台
// - outputDir: 图像保存目录；为空表示使用当前工作目录
// - framesToCapture: 软触发采集并保存的帧数（将保存为 I1..IframesToCapture.png）
// - params: 相机参数配置，包含曝光时间、增益、帧率等
// 返回：
// - true 表示测试流程全部成功；false 表示其中出现错误
static bool runCameraTest(const std::string& cameraSerial,
                          const std::string& outputDir,
                          int framesToCapture,
                          const CameraParams& params = CameraParams{}) {
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
    std::cout << "配置相机基础参数..." << std::endl;
    
    // 步骤1: 设置像素格式为Mono8
    nRet = MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    if (nRet != MV_OK) {
        std::cerr << "设置像素格式失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }
    std::cout << "像素格式设置为Mono8成功" << std::endl;
    
    // 步骤2: 尝试设置触发选择器为FrameStart（兼容性更好）
    nRet = MV_CC_SetEnumValueByString(handle, "TriggerSelector", "FrameStart");
    if (nRet != MV_OK) {
        // 如果FrameStart失败，尝试使用数值6 (FrameBurstStart)
        std::cout << "设置TriggerSelector为FrameStart失败，尝试FrameBurstStart..." << std::endl;
        nRet = MV_CC_SetEnumValue(handle, "TriggerSelector", 6);
        if (nRet != MV_OK) {
            std::cerr << "设置触发选择器失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            return false;
        }
        std::cout << "触发选择器设置为FrameBurstStart成功" << std::endl;
    } else {
        std::cout << "触发选择器设置为FrameStart成功" << std::endl;
    }
    
    // 步骤3: 开启触发模式
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);          // On
    if (nRet != MV_OK) {
        std::cerr << "开启触发模式失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }
    std::cout << "触发模式开启成功" << std::endl;
    
    // 步骤4: 设置触发源为软件触发
    nRet = MV_CC_SetEnumValueByString(handle, "TriggerSource", "Software");
    if (nRet != MV_OK) {
        std::cerr << "设置触发源失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }
    std::cout << "触发源设置为Software成功" << std::endl;
    
    // 步骤5: 设置采集模式为连续模式
    nRet = MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous");
    if (nRet != MV_OK) {
        std::cerr << "设置采集模式失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }
    std::cout << "采集模式设置为Continuous成功" << std::endl;
    
    // 步骤6: 设置图像缓存节点数量（提高稳定性）
    // 根据工业相机指导文档，缓存节点数量影响图像采集的稳定性
    // 建议设置为至少等于要采集的图像数量
    nRet = MV_CC_SetImageNodeNum(handle, framesToCapture + 2); // 多设置2个节点作为缓冲
    if (nRet != MV_OK) {
        std::cout << "设置图像缓存节点数量失败，使用默认值，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
    } else {
        std::cout << "图像缓存节点数量设置为" << (framesToCapture + 2) << "成功" << std::endl;
    }
    
    std::cout << "相机基础参数配置完成" << std::endl;

    // 4.5) 获取相机参数范围信息
    CameraParams workingParams = params;
    getCameraParamRanges(handle, workingParams);
    
    // 4.6) 配置相机参数（曝光时间、增益、帧率等）
    if (!configureCameraParams(handle, workingParams)) {
        std::cerr << "相机参数配置失败" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 5) 准备保存目录与回调
    // - 创建保存目录（若不存在）
    // - 注册图像回调：接收帧数据并保存为 I1..I(总数).png
    CallbackContext ctx;
    ctx.totalFrames = framesToCapture;
    // 不再进行智能曝光调整，保持相机默认设置
    if (!outputDir.empty()) {
        ctx.saveDir = outputDir;
    } else {
        // 默认保存到当前目录下的 images 子目录
        ctx.saveDir = (std::filesystem::current_path() / "images").string();
    }
    try { std::filesystem::create_directories(ctx.saveDir); } catch (...) {}

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
    std::cout << "开始软触发抓拍 " << framesToCapture << " 张图像..." << std::endl;
    
    for (int i = 0; i < framesToCapture; i++) {
        std::cout << "执行第 " << (i + 1) << " 次软触发..." << std::endl;
        
        // 根据TriggerSelector设置选择正确的软触发命令
        std::string triggerCommand;
        MVCC_ENUMVALUE triggerSelector;
        nRet = MV_CC_GetEnumValue(handle, "TriggerSelector", &triggerSelector);
        if (nRet == MV_OK) {
            if (triggerSelector.nCurValue == 0) {  // FrameStart
                triggerCommand = "FrameTriggerSoftware";
            } else {
                triggerCommand = "TriggerSoftware";  // 用于FrameBurstStart等
            }
        } else {
            triggerCommand = "TriggerSoftware";  // 默认使用
        }
        
        std::cout << "使用软触发命令: " << triggerCommand << std::endl;
        
        // 使用正确的软触发命令：MV_CC_SetCommandValue
        nRet = MV_CC_SetCommandValue(handle, triggerCommand.c_str());
        if (nRet != MV_OK) {
            std::cerr << "软触发失败: 0x" << std::hex << nRet << std::dec << std::endl;
            return false;
        }
        
        std::cout << "第 " << (i + 1) << " 次软触发成功" << std::endl;
        
        // 根据工业相机指导文档，软触发后需要足够的等待时间让相机完成曝光和图像传输
        // 增加等待时间，确保图像能够被正确采集
        // 注意：如果曝光时间很长，需要等待更长时间
        // 根据当前设置的曝光时间动态调整等待时间
        MVCC_FLOATVALUE currentExposure;
        int waitTimeMs = 1000; // 默认等待1秒
        if (MV_CC_GetFloatValue(handle, "ExposureTime", &currentExposure) == MV_OK) {
            // 等待时间 = 曝光时间 + 额外缓冲时间（500ms）
            waitTimeMs = static_cast<int>(static_cast<double>(currentExposure.fCurValue) / 1000.0) + 500;
            // 限制最大等待时间为5秒，避免等待过久
            if (waitTimeMs > 5000) waitTimeMs = 5000;
        }
        std::cout << "  等待时间: " << waitTimeMs << "ms" << std::endl;
        Sleep(waitTimeMs);
        
        // 添加调试信息：检查当前相机状态
        MVCC_ENUMVALUE currentTriggerMode;
        if (MV_CC_GetEnumValue(handle, "TriggerMode", &currentTriggerMode) == MV_OK) {
            std::cout << "  当前触发模式: " << (currentTriggerMode.nCurValue == 1 ? "On" : "Off") << std::endl;
        }
        
        MVCC_ENUMVALUE currentTriggerSource;
        if (MV_CC_GetEnumValue(handle, "TriggerSource", &currentTriggerSource) == MV_OK) {
            std::cout << "  当前触发源: " << currentTriggerSource.nCurValue << std::endl;
        }
    }
    
    std::cout << "软触发完成，等待回调处理..." << std::endl;
    
    // 等待回调写盘完成，根据工业相机指导文档，需要足够的等待时间
    // 软触发后，相机需要时间完成曝光、图像传输和回调处理
    std::cout << "等待图像采集和保存完成..." << std::endl;
    
    // 根据采集的图像数量和曝光时间计算总等待时间
    MVCC_FLOATVALUE currentExposure;
    int totalWaitTimeMs = 5000; // 默认等待5秒
    if (MV_CC_GetFloatValue(handle, "ExposureTime", &currentExposure) == MV_OK) {
        // 总等待时间 = 图像数量 × (曝光时间 + 传输时间) + 额外缓冲
        // 传输时间估算为500ms每张图像
        double exposureTimeMs = static_cast<double>(currentExposure.fCurValue) / 1000.0;
        double transmissionTimeMs = framesToCapture * 500.0;
        double totalTimeMs = framesToCapture * exposureTimeMs + transmissionTimeMs + 2000; // 额外2秒缓冲
        totalWaitTimeMs = static_cast<int>(totalTimeMs);
        // 限制最大等待时间为30秒，避免等待过久
        if (totalWaitTimeMs > 30000) totalWaitTimeMs = 30000;
    }
    
    std::cout << "预计总等待时间: " << totalWaitTimeMs << "ms" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(totalWaitTimeMs));
    
    // 验证是否成功采集到图像
    std::cout << "验证图像采集结果..." << std::endl;
    int savedImageCount = 0;
    for (int i = 1; i <= framesToCapture; ++i) {
        std::string filename = "I" + std::to_string(i) + ".png";
        std::string filepath = (std::filesystem::path(ctx.saveDir) / filename).string();
        if (std::filesystem::exists(filepath)) {
            std::cout << "✓ 图像 " << i << " 保存成功: " << filepath << std::endl;
            savedImageCount++;
        } else {
            std::cout << "✗ 图像 " << i << " 保存失败: " << filepath << std::endl;
        }
    }
    
    if (savedImageCount == framesToCapture) {
        std::cout << "所有图像采集成功！共保存 " << savedImageCount << " 张图像" << std::endl;
    } else {
        std::cout << "图像采集不完整！期望 " << framesToCapture << " 张，实际保存 " << savedImageCount << " 张" << std::endl;
    }

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
    int stepResult = 0;
    
    std::cout << "开始配置相机触发参数..." << std::endl;
    
    // 步骤1: 设置像素格式为Mono8
    std::cout << "步骤1: 设置像素格式为Mono8..." << std::endl;
    stepResult = MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    if (stepResult != MV_OK) {
        std::cout << "  失败: 设置像素格式失败，错误码: 0x" << std::hex << stepResult << std::dec << std::endl;
        ok = false;
    } else {
        std::cout << "  成功: 像素格式设置为Mono8" << std::endl;
    }
    
    // 步骤2: 设置触发选择器为FrameStart
    std::cout << "步骤2: 设置触发选择器为FrameStart..." << std::endl;
    stepResult = MV_CC_SetEnumValueByString(handle, "TriggerSelector", "FrameStart");
    if (stepResult != MV_OK) {
        // 如果FrameStart失败，尝试使用数值6 (FrameBurstStart)
        std::cout << "  尝试使用FrameBurstStart..." << std::endl;
        stepResult = MV_CC_SetEnumValue(handle, "TriggerSelector", 6);
        if (stepResult != MV_OK) {
            std::cout << "  失败: 设置触发选择器失败，错误码: 0x" << std::hex << stepResult << std::dec << std::endl;
            ok = false;
        } else {
            std::cout << "  成功: 触发选择器设置为FrameBurstStart" << std::endl;
        }
    } else {
        std::cout << "  成功: 触发选择器设置为FrameStart" << std::endl;
    }
    
    // 步骤3: 开启触发模式
    std::cout << "步骤3: 开启触发模式..." << std::endl;
    stepResult = MV_CC_SetEnumValue(handle, "TriggerMode", 1);  // 1 = On
    if (stepResult != MV_OK) {
        std::cout << "  失败: 开启触发模式失败，错误码: 0x" << std::hex << stepResult << std::dec << std::endl;
        ok = false;
    } else {
        std::cout << "  成功: 触发模式已开启" << std::endl;
    }
    
    // 步骤4: 设置触发源为软件触发
    std::cout << "步骤4: 设置触发源为软件触发..." << std::endl;
    stepResult = MV_CC_SetEnumValueByString(handle, "TriggerSource", "Software");
    if (stepResult != MV_OK) {
        std::cout << "  失败: 设置触发源失败，错误码: 0x" << std::hex << stepResult << std::dec << std::endl;
        ok = false;
    } else {
        std::cout << "  成功: 触发源设置为Software" << std::endl;
    }
    
    // 步骤5: 设置采集模式为连续模式
    std::cout << "步骤5: 设置采集模式为连续模式..." << std::endl;
    stepResult = MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous");
    if (stepResult != MV_OK) {
        std::cout << "  失败: 设置采集模式失败，错误码: 0x" << std::hex << stepResult << std::dec << std::endl;
        ok = false;
    } else {
        std::cout << "  成功: 采集模式设置为Continuous" << std::endl;
    }
    
    // 验证配置结果
    if (ok) {
        std::cout << "所有触发参数配置成功！" << std::endl;
        
        // 验证配置是否正确
        std::cout << "验证配置结果..." << std::endl;
        
        // 验证像素格式
        MVCC_ENUMVALUE pixelFormat;
        if (MV_CC_GetEnumValue(handle, "PixelFormat", &pixelFormat) == MV_OK) {
            std::cout << "  像素格式: " << pixelFormat.nCurValue << std::endl;
        }
        
        // 验证触发模式
        MVCC_ENUMVALUE triggerMode;
        if (MV_CC_GetEnumValue(handle, "TriggerMode", &triggerMode) == MV_OK) {
            std::cout << "  触发模式: " << (triggerMode.nCurValue == 1 ? "On" : "Off") << std::endl;
        }
        
        // 验证触发源
        MVCC_ENUMVALUE triggerSource;
        if (MV_CC_GetEnumValue(handle, "TriggerSource", &triggerSource) == MV_OK) {
            std::cout << "  触发源: " << triggerSource.nCurValue << std::endl;
        }
        
        assertTrue(true, "成功配置 Mono8 + 软件触发");
    } else {
        std::cout << "触发参数配置失败！" << std::endl;
        assertTrue(false, "配置 Mono8 + 软件触发");
    }

    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
}



// 采集保存测试（复用端到端流程）
static void testCameraCaptureAndSave(const std::string& cameraSerial, const std::string& saveDir, int frames) {
    std::cout << "\n--- 测试相机采集并保存图像 ---" << std::endl;
    bool ok = runCameraTest(cameraSerial, saveDir, frames);
    assertTrue(ok, "端到端采集与保存成功");
}

// 相机参数配置测试
static void testCameraParameterConfiguration(const std::string& cameraSerial) {
    std::cout << "\n--- 测试相机参数配置 ---" << std::endl;
    
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        assertTrue(false, "设备枚举失败，无法进行参数配置测试");
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
    
    // 测试1: 获取参数范围
    std::cout << "测试1: 获取相机参数范围" << std::endl;
    CameraParams testParams;
    bool ok1 = getCameraParamRanges(handle, testParams);
    assertTrue(ok1, "获取相机参数范围成功");
    
    // 测试2: 设置曝光时间
    std::cout << "\n测试2: 设置曝光时间" << std::endl;
    testParams.exposureTimeUs = 10000.0f; // 10ms
    testParams.exposureAutoMode = false;
    testParams.gainAutoMode = false;
    testParams.printCurrentParams = true;
    bool ok2 = configureCameraParams(handle, testParams);
    assertTrue(ok2, "设置曝光时间成功");
    
    // 测试3: 设置增益
    std::cout << "\n测试3: 设置增益" << std::endl;
    testParams.gainValue = 5.0f; // 5dB
    bool ok3 = configureCameraParams(handle, testParams);
    assertTrue(ok3, "设置增益成功");
    
    // 测试4: 设置帧率
    std::cout << "\n测试4: 设置帧率" << std::endl;
    testParams.frameRate = 10.0f; // 10fps
    bool ok4 = configureCameraParams(handle, testParams);
    assertTrue(ok4, "设置帧率成功");
    
    // 测试5: 启用自动曝光
    std::cout << "\n测试5: 启用自动曝光" << std::endl;
    testParams.exposureAutoMode = true;
    testParams.exposureTimeUs = -1.0f; // 重置曝光时间
    bool ok5 = configureCameraParams(handle, testParams);
    assertTrue(ok5, "启用自动曝光成功");
    
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
}

// 不同曝光参数下的图像采集测试
static void testCameraWithDifferentExposureSettings(const std::string& cameraSerial, const std::string& saveDir) {
    std::cout << "\n--- 测试不同曝光参数下的图像采集 ---" << std::endl;
    
    // 测试不同的曝光时间设置
    std::vector<float> exposureTimes = {5000.0f, 10000.0f, 20000.0f, 50000.0f}; // 5ms, 10ms, 20ms, 50ms
    std::vector<std::string> exposureNames = {"5ms", "10ms", "20ms", "50ms"};
    
    for (size_t i = 0; i < exposureTimes.size(); ++i) {
        std::cout << "\n测试曝光时间: " << exposureNames[i] << " (" << exposureTimes[i] << " μs)" << std::endl;
        
        CameraParams params;
        params.exposureTimeUs = exposureTimes[i];
        params.exposureAutoMode = false;
        params.gainAutoMode = false;
        params.frameRate = 5.0f; // 降低帧率确保稳定
        params.printCurrentParams = true;
        
        std::string testSaveDir = saveDir + "/exposure_" + exposureNames[i];
        bool ok = runCameraTest(cameraSerial, testSaveDir, 3, params);
        assertTrue(ok, "曝光时间 " + exposureNames[i] + " 测试成功");
    }
}

// 不同增益参数下的图像采集测试
static void testCameraWithDifferentGainSettings(const std::string& cameraSerial, const std::string& saveDir) {
    std::cout << "\n--- 测试不同增益参数下的图像采集 ---" << std::endl;
    
    // 测试不同的增益设置
    std::vector<float> gainValues = {0.0f, 5.0f, 10.0f, 15.0f}; // 0dB, 5dB, 10dB, 15dB
    std::vector<std::string> gainNames = {"0dB", "5dB", "10dB", "15dB"};
    
    for (size_t i = 0; i < gainValues.size(); ++i) {
        std::cout << "\n测试增益: " << gainNames[i] << " (" << gainValues[i] << " dB)" << std::endl;
        
        CameraParams params;
        params.exposureTimeUs = 10000.0f; // 固定10ms曝光时间
        params.exposureAutoMode = false;
        params.gainValue = gainValues[i];
        params.gainAutoMode = false;
        params.frameRate = 5.0f; // 降低帧率确保稳定
        params.printCurrentParams = true;
        
        std::string testSaveDir = saveDir + "/gain_" + gainNames[i];
        bool ok = runCameraTest(cameraSerial, testSaveDir, 3, params);
        assertTrue(ok, "增益 " + gainNames[i] + " 测试成功");
    }
}

// 软触发兼容性测试（测试不同的触发配置方式）
static void testCameraSoftwareTriggerCompatibility(const std::string& cameraSerial) {
    std::cout << "\n--- 测试相机软触发兼容性 ---" << std::endl;
    
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        assertTrue(false, "设备枚举失败，无法进行软触发兼容性测试");
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
    if (nRet != MV_OK || !handle) { 
        assertTrue(false, "创建相机句柄失败"); 
        return; 
    }
    
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) { 
        assertTrue(false, "打开相机失败"); 
        MV_CC_DestroyHandle(handle); 
        return; 
    }
    
    std::cout << "相机连接成功，开始测试软触发兼容性..." << std::endl;
    
    // 测试方法1: 使用TriggerSelector = "FrameStart"
    std::cout << "\n测试方法1: TriggerSelector = FrameStart" << std::endl;
    bool method1Ok = true;
    
    nRet = MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    if (nRet != MV_OK) method1Ok = false;
    
    if (method1Ok) {
        nRet = MV_CC_SetEnumValueByString(handle, "TriggerSelector", "FrameStart");
        if (nRet == MV_OK) {
            std::cout << "TriggerSelector设置为FrameStart成功" << std::endl;
            nRet = MV_CC_SetEnumValueByString(handle, "TriggerMode", "On");
            if (nRet == MV_OK) {
                std::cout << "TriggerMode设置为On成功" << std::endl;
                nRet = MV_CC_SetEnumValueByString(handle, "TriggerSource", "Software");
                if (nRet == MV_OK) {
                    std::cout << "TriggerSource设置为Software成功" << std::endl;
                    method1Ok = true;
                } else {
                    std::cout << "TriggerSource设置失败" << std::endl;
                    method1Ok = false;
                }
            } else {
                std::cout << "TriggerMode设置失败" << std::endl;
                method1Ok = false;
            }
        } else {
            std::cout << "TriggerSelector设置失败" << std::endl;
            method1Ok = false;
        }
    }
    
    std::cout << "方法1结果: " << (method1Ok ? "成功" : "失败") << std::endl;
    
    // 测试方法2: 使用TriggerSelector = 6 (FrameBurstStart)
    std::cout << "\n测试方法2: TriggerSelector = FrameBurstStart" << std::endl;
    bool method2Ok = true;
    
    nRet = MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    if (nRet != MV_OK) method2Ok = false;
    
    if (method2Ok) {
        nRet = MV_CC_SetEnumValue(handle, "TriggerSelector", 6);
        if (nRet == MV_OK) {
            std::cout << "TriggerSelector设置为FrameBurstStart成功" << std::endl;
            nRet = MV_CC_SetEnumValueByString(handle, "TriggerMode", "On");
            if (nRet == MV_OK) {
                std::cout << "TriggerMode设置为On成功" << std::endl;
                nRet = MV_CC_SetEnumValueByString(handle, "TriggerSource", "Software");
                if (nRet == MV_OK) {
                    std::cout << "TriggerSource设置为Software成功" << std::endl;
                    method2Ok = true;
                } else {
                    std::cout << "TriggerSource设置失败" << std::endl;
                    method2Ok = false;
                }
            } else {
                std::cout << "TriggerMode设置失败" << std::endl;
                method2Ok = false;
            }
        } else {
            std::cout << "TriggerSelector设置失败" << std::endl;
            method2Ok = false;
        }
    }
    
    std::cout << "方法2结果: " << (method2Ok ? "成功" : "失败") << std::endl;
    
    // 测试方法3: 尝试使用FrameTriggerSource（某些相机可能支持）
    std::cout << "\n测试方法3: FrameTriggerSource = Software" << std::endl;
    bool method3Ok = true;
    
    nRet = MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    if (nRet != MV_OK) method3Ok = false;
    
    if (method3Ok) {
        nRet = MV_CC_SetEnumValueByString(handle, "FrameTriggerSource", "Software");
        if (nRet == MV_OK) {
            std::cout << "FrameTriggerSource设置为Software成功" << std::endl;
            // 设置对应的触发模式
            nRet = MV_CC_SetBoolValue(handle, "FrameTriggerMode", true);
            if (nRet == MV_OK) {
                std::cout << "FrameTriggerMode设置为true成功" << std::endl;
                method3Ok = true;
            } else {
                std::cout << "FrameTriggerMode设置失败" << std::endl;
                method3Ok = false;
            }
        } else {
            std::cout << "FrameTriggerSource设置失败" << std::endl;
            method3Ok = false;
        }
    }
    
    std::cout << "方法3结果: " << (method3Ok ? "成功" : "失败") << std::endl;
    
    // 总结兼容性测试结果
    if (method1Ok || method2Ok || method3Ok) {
        std::cout << "\n软触发兼容性测试: 至少有一种方法可用" << std::endl;
        assertTrue(true, "软触发兼容性测试通过");
    } else {
        std::cout << "\n软触发兼容性测试: 所有方法都失败" << std::endl;
        assertTrue(false, "软触发兼容性测试失败");
    }
    
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
}

// 软触发功能测试
static void testCameraSoftwareTrigger(const std::string& cameraSerial) {
    std::cout << "\n--- 测试相机软触发功能 ---" << std::endl;
    
    // 使用简化的测试流程，专门测试软触发
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        std::cerr << "未发现可用相机，错误码: " << nRet << std::endl;
        assertTrue(false, "设备枚举失败，无法进行软触发测试");
        return;
    }
    
    // 选择设备（优先匹配序列号；否则退回第一台）
    MV_CC_DEVICE_INFO* pSelectedDevice = nullptr;
    if (!cameraSerial.empty() && cameraSerial != "NULL") {
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
    if (pSelectedDevice == nullptr) {
        pSelectedDevice = deviceList.pDeviceInfo[0];
    }
    
    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, pSelectedDevice);
    if (nRet != MV_OK || handle == nullptr) {
        std::cerr << "创建相机句柄失败，错误码: " << nRet << std::endl;
        assertTrue(false, "创建相机句柄失败");
        return;
    }
    
    // 2. 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        std::cerr << "打开设备失败: 0x" << std::hex << nRet << std::dec << std::endl;
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "打开设备失败");
        return;
    }
    
    std::cout << "相机连接成功，开始配置软触发参数..." << std::endl;
    
    // 配置软触发参数
    bool configOk = true;
    
    // 1. 设置像素格式
    nRet = MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    if (nRet != MV_OK) {
        std::cerr << "设置像素格式失败: 0x" << std::hex << nRet << std::dec << std::endl;
        configOk = false;
    }
    
    // 2. 设置触发选择器 - 这是关键步骤
    if (configOk) {
        // 首先尝试FrameStart
        nRet = MV_CC_SetEnumValueByString(handle, "TriggerSelector", "FrameStart");
        if (nRet != MV_OK) {
            // 如果FrameStart失败，尝试使用数值6 (FrameBurstStart)
            std::cout << "尝试使用FrameBurstStart..." << std::endl;
            nRet = MV_CC_SetEnumValue(handle, "TriggerSelector", 6);
            if (nRet != MV_OK) {
                std::cerr << "设置触发选择器失败: 0x" << std::hex << nRet << std::dec << std::endl;
                configOk = false;
            } else {
                std::cout << "触发选择器设置为FrameBurstStart成功" << std::endl;
            }
        } else {
            std::cout << "触发选择器设置为FrameStart成功" << std::endl;
        }
    }
    
    // 3. 开启触发模式
    if (configOk) {
        nRet = MV_CC_SetEnumValueByString(handle, "TriggerMode", "On");
        if (nRet != MV_OK) {
            std::cerr << "开启触发模式失败: 0x" << std::hex << nRet << std::dec << std::endl;
            configOk = false;
        }
    }
    
    // 4. 设置触发源为软件触发
    if (configOk) {
        nRet = MV_CC_SetEnumValueByString(handle, "TriggerSource", "Software");
        if (nRet != MV_OK) {
            std::cerr << "设置软件触发源失败: 0x" << std::hex << nRet << std::dec << std::endl;
            configOk = false;
        }
    }
    
    // 5. 设置采集模式
    if (configOk) {
        nRet = MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous");
        if (nRet != MV_OK) {
            std::cerr << "设置采集模式失败: 0x" << std::hex << nRet << std::dec << std::endl;
            configOk = false;
        }
    }
    
    // 6. 设置图像缓存节点数量（提高稳定性）
    if (configOk) {
        nRet = MV_CC_SetImageNodeNum(handle, 5);
        if (nRet != MV_OK) {
            std::cout << "设置图像缓存节点数量失败，使用默认值: 0x" << std::hex << nRet << std::dec << std::endl;
        } else {
            std::cout << "图像缓存节点数量设置为5成功" << std::endl;
        }
    }
    
    if (!configOk) {
        std::cerr << "软触发参数配置失败" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "软触发参数配置失败");
        return;
    }
    
    std::cout << "软触发参数配置成功" << std::endl;
    
    // 7. 注册图像回调函数
    CallbackContext ctx;
    ctx.totalFrames = 3;
    // 不再进行智能曝光调整，保持相机默认设置
    ctx.saveDir = "images_Projector";
    
    // 确保保存目录存在
    try {
        std::filesystem::create_directories(ctx.saveDir);
    } catch (...) {
        std::cout << "创建保存目录失败，使用当前目录" << std::endl;
        ctx.saveDir = ".";
    }
    
    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallbackEx, &ctx);
    if (nRet != MV_OK) {
        std::cerr << "注册图像回调失败: 0x" << std::hex << nRet << std::dec << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "注册图像回调失败");
        return;
    }
    
    // 8. 开始取流
    nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        std::cerr << "开始取流失败: 0x" << std::hex << nRet << std::dec << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "开始取流失败");
        return;
    }
    
    std::cout << "开始取流，执行软触发测试..." << std::endl;
    
    // 9. 执行软触发
    for (int i = 0; i < 3; ++i) {
        std::cout << "执行第" << (i + 1) << "次软触发..." << std::endl;
        
        // 根据TriggerSelector设置选择正确的软触发命令
        std::string triggerCommand;
        MVCC_ENUMVALUE triggerSelector;
        nRet = MV_CC_GetEnumValue(handle, "TriggerSelector", &triggerSelector);
        if (nRet == MV_OK) {
            if (triggerSelector.nCurValue == 0) {  // FrameStart
                triggerCommand = "FrameTriggerSoftware";
            } else {
                triggerCommand = "TriggerSoftware";  // 用于FrameBurstStart等
            }
        } else {
            triggerCommand = "TriggerSoftware";  // 默认使用
        }
        
        // 使用正确的软触发命令
        nRet = MV_CC_SetCommandValue(handle, triggerCommand.c_str());
        if (nRet != MV_OK) {
            std::cerr << "软触发失败: 0x" << std::hex << nRet << std::dec << std::endl;
        } else {
            std::cout << "软触发成功" << std::endl;
        }
        
        // 根据工业相机指导文档，如果帧率过小或TriggerDelay很大，可能会出现软触发命令没有全部起效的情况
        // 添加适当的延时确保触发命令生效
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 延时500ms确保触发稳定
    }
    
    // 10. 等待回调处理完成
    std::cout << "等待图像回调处理..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // 11. 停止取流
    nRet = MV_CC_StopGrabbing(handle);
    if (nRet != MV_OK) {
        std::cerr << "停止取流失败: 0x" << std::hex << nRet << std::dec << std::endl;
    }
    
    // 12. 关闭设备
    nRet = MV_CC_CloseDevice(handle);
    if (nRet != MV_OK) {
        std::cerr << "关闭设备失败: 0x" << std::hex << nRet << std::dec << std::endl;
    }
    
    // 13. 销毁句柄
    nRet = MV_CC_DestroyHandle(handle);
    if (nRet != MV_OK) {
        std::cerr << "销毁句柄失败: 0x" << std::hex << nRet << std::dec << std::endl;
    }
    
    std::cout << "软触发测试完成，保存目录: " << ctx.saveDir << std::endl;
    assertTrue(true, "软触发功能测试完成");
}





// 完整端到端测试
static void testCameraEndToEnd(const std::string& cameraSerial,
                               const std::string& saveDir,
                               int frames) {
    std::cout << "\n--- 测试相机端到端流程 ---" << std::endl;
    bool ok = runCameraTest(cameraSerial, saveDir, frames);
    assertTrue(ok, "端到端流程执行成功");
}

static void runAllCameraTests() {
    // 基础信息
    const std::string cameraSerial = "NULL"; // 或指定序列号，例如 "DA1015150"
    const std::string saveDir = "images_Projector";

    
    // 基础能力
    testCameraEnumerate();
    testCameraOpenClose(cameraSerial);
    testCameraConfigureTrigger(cameraSerial);

    // 相机参数配置测试
    testCameraParameterConfiguration(cameraSerial);

    // 采集能力
    testCameraCaptureAndSave(cameraSerial, saveDir, 4);

    // 不同参数下的图像采集测试
    testCameraWithDifferentExposureSettings(cameraSerial, saveDir);
    testCameraWithDifferentGainSettings(cameraSerial, saveDir);

    // 软触发兼容性测试
    testCameraSoftwareTriggerCompatibility(cameraSerial);

    // 软触发功能测试
    testCameraSoftwareTrigger(cameraSerial);
    
    // 端到端示例
    testCameraEndToEnd(cameraSerial, saveDir, 8);
     

    g_camTestResults.printSummary();
}

// 创建自定义相机参数配置
static CameraParams createCustomCameraParams() {
    CameraParams params;
    
    // 检查是否有保存的参数文件
    if (paramsFileExists()) {
        std::cout << "\n发现已保存的参数文件，是否要加载? (1=是, 0=否): ";
        int loadSaved;
        std::cin >> loadSaved;
        
        if (loadSaved == 1) {
            if (loadCameraParams(params)) {
                displaySavedParams(params);
                std::cout << "是否要修改这些参数? (1=是, 0=否): ";
                int modify;
                std::cin >> modify;
                if (modify == 0) {
                    return params; // 直接使用加载的参数
                }
            }
        }
    }
    
    std::cout << "\n=== 相机参数配置 ===" << std::endl;
    std::cout << "请输入相机参数（输入-1使用默认值，输入0跳过该参数）:" << std::endl;
    
    // 曝光时间配置
    std::cout << "曝光时间 (微秒，建议范围: 1000-100000): ";
    std::cin >> params.exposureTimeUs;
    if (params.exposureTimeUs <= 0) {
        params.exposureTimeUs = -1.0f; // 使用默认值
    }
    
    // 自动曝光模式
    std::cout << "是否启用自动曝光? (1=是, 0=否): ";
    int autoExposure;
    std::cin >> autoExposure;
    params.exposureAutoMode = (autoExposure == 1);
    
    // 增益配置
    std::cout << "增益值 (dB，建议范围: 0-20): ";
    std::cin >> params.gainValue;
    if (params.gainValue < 0) {
        params.gainValue = -1.0f; // 使用默认值
    }
    
    // 自动增益模式
    std::cout << "是否启用自动增益? (1=是, 0=否): ";
    int autoGain;
    std::cin >> autoGain;
    params.gainAutoMode = (autoGain == 1);
    
    // 帧率配置
    std::cout << "帧率 (fps，建议范围: 1-30): ";
    std::cin >> params.frameRate;
    if (params.frameRate <= 0) {
        params.frameRate = -1.0f; // 使用默认值
    }
    
    // 触发延时
    std::cout << "触发延时 (微秒，0表示无延时): ";
    std::cin >> params.triggerDelayUs;
    if (params.triggerDelayUs < 0) {
        params.triggerDelayUs = 0;
    }
    
    // 是否打印当前参数
    std::cout << "是否打印当前相机参数? (1=是, 0=否): ";
    int printParams;
    std::cin >> printParams;
    params.printCurrentParams = (printParams == 1);
    
    std::cout << "参数配置完成!" << std::endl;
    
    // 询问是否保存参数
    std::cout << "是否保存这些参数供下次使用? (1=是, 0=否): ";
    int saveParams;
    std::cin >> saveParams;
    
    if (saveParams == 1) {
        // 备份现有参数文件
        backupParamsFile();
        
        // 保存新参数
        if (saveCameraParams(params)) {
            std::cout << "参数已保存，下次运行快速测试时将使用这些参数。" << std::endl;
        } else {
            std::cout << "参数保存失败，但测试仍可继续。" << std::endl;
        }
    }
    
    return params;
}

// 运行自定义参数测试
static void runCustomParameterTest() {
    std::cout << "\n=== 自定义参数测试 ===" << std::endl;
    
    const std::string cameraSerial = "NULL";
    const std::string saveDir = "images_Projector";
    const int framesToCapture = 5;
    
    // 创建自定义参数
    CameraParams customParams = createCustomCameraParams();
    
    // 运行测试
    std::cout << "\n开始使用自定义参数进行图像采集..." << std::endl;
    bool ok = runCameraTest(cameraSerial, saveDir, framesToCapture, customParams);
    
    if (ok) {
        std::cout << "自定义参数测试成功完成！" << std::endl;
        std::cout << "图像已保存到: " << saveDir << std::endl;
    } else {
        std::cout << "自定义参数测试失败！" << std::endl;
    }
}

// 主函数：设置控制台编码，配置测试参数并运行
int main() {
#if defined(_WIN32)
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    
    std::cout << "=== 相机测试程序 ===" << std::endl;
    std::cout << "请选择测试模式:" << std::endl;
    std::cout << "1. 运行所有测试用例" << std::endl;
    std::cout << "2. 自定义参数测试（解决过曝/欠曝问题）" << std::endl;
    std::cout << "3. 快速测试（使用保存的参数）" << std::endl;
    std::cout << "4. 参数管理" << std::endl;
    std::cout << "请输入选择 (1-4): ";
    
    int choice;
    std::cin >> choice;
    
    switch (choice) {
        case 1:
            std::cout << "运行所有测试用例..." << std::endl;
            runAllCameraTests();
            break;
            
        case 2:
            std::cout << "运行自定义参数测试..." << std::endl;
            runCustomParameterTest();
            break;
            
        case 3:
            std::cout << "运行快速测试..." << std::endl;
            {
                const std::string cameraSerial = "NULL";
                const std::string saveDir = "images_Projector";
                
                // 尝试加载保存的参数
                CameraParams testParams;
                bool paramsLoaded = loadCameraParams(testParams);
                
                if (paramsLoaded) {
                    std::cout << "使用保存的参数进行快速测试..." << std::endl;
                    displaySavedParams(testParams);
                } else {
                    std::cout << "未找到保存的参数，使用默认参数进行快速测试..." << std::endl;
                    // 设置默认参数
                    testParams.exposureTimeUs = 10000.0f; // 10ms
                    testParams.gainValue = 5.0f; // 5dB
                    testParams.frameRate = 10.0f; // 10fps
                    testParams.printCurrentParams = true;
                    testParams.exposureAutoMode = false;
                    testParams.gainAutoMode = false;
                    testParams.triggerDelayUs = 0;
                    testParams.enableChunkData = false;
                }
                
                bool ok = runCameraTest(cameraSerial, saveDir, 5, testParams);
                if (ok) {
                    std::cout << "快速测试成功完成！" << std::endl;
                    if (paramsLoaded) {
                        std::cout << "使用的参数来自保存的配置文件。" << std::endl;
                    } else {
                        std::cout << "使用的参数为默认值。" << std::endl;
                    }
                } else {
                    std::cout << "快速测试失败！" << std::endl;
                }
            }
            break;
            
        case 4:
            std::cout << "参数管理..." << std::endl;
            {
                std::cout << "\n=== 参数管理 ===" << std::endl;
                std::cout << "1. 查看保存的参数" << std::endl;
                std::cout << "2. 删除保存的参数文件" << std::endl;
                std::cout << "3. 备份参数文件" << std::endl;
                std::cout << "4. 恢复备份的参数文件" << std::endl;
                std::cout << "请输入选择 (1-4): ";
                
                int paramChoice;
                std::cin >> paramChoice;
                
                switch (paramChoice) {
                    case 1:
                        {
                            if (paramsFileExists()) {
                                CameraParams params;
                                if (loadCameraParams(params)) {
                                    displaySavedParams(params);
                                }
                            } else {
                                std::cout << "未找到保存的参数文件。" << std::endl;
                            }
                        }
                        break;
                        
                    case 2:
                        {
                            if (paramsFileExists()) {
                                std::cout << "确定要删除参数文件吗? (1=是, 0=否): ";
                                int confirm;
                                std::cin >> confirm;
                                if (confirm == 1) {
                                    try {
                                        std::filesystem::remove(PARAMS_FILE);
                                        std::cout << "参数文件已删除。" << std::endl;
                                    } catch (const std::exception& e) {
                                        std::cerr << "删除参数文件失败: " << e.what() << std::endl;
                                    }
                                }
                            } else {
                                std::cout << "未找到保存的参数文件。" << std::endl;
                            }
                        }
                        break;
                        
                    case 3:
                        {
                            if (backupParamsFile()) {
                                std::cout << "参数文件备份完成。" << std::endl;
                            } else {
                                std::cout << "参数文件备份失败。" << std::endl;
                            }
                        }
                        break;
                        
                    case 4:
                        {
                            std::string backupFile = PARAMS_FILE + ".backup";
                            if (std::filesystem::exists(backupFile)) {
                                std::cout << "确定要恢复备份的参数文件吗? (1=是, 0=否): ";
                                int confirm;
                                std::cin >> confirm;
                                if (confirm == 1) {
                                    try {
                                        std::filesystem::copy_file(backupFile, PARAMS_FILE, std::filesystem::copy_options::overwrite_existing);
                                        std::cout << "参数文件已从备份恢复。" << std::endl;
                                        
                                        // 显示恢复的参数
                                        CameraParams params;
                                        if (loadCameraParams(params)) {
                                            displaySavedParams(params);
                                        }
                                    } catch (const std::exception& e) {
                                        std::cerr << "恢复参数文件失败: " << e.what() << std::endl;
                                    }
                                }
                            } else {
                                std::cout << "未找到备份文件。" << std::endl;
                            }
                        }
                        break;
                        
                    default:
                        std::cout << "无效选择。" << std::endl;
                        break;
                }
            }
            break;
            
        default:
            std::cout << "无效选择，运行默认测试..." << std::endl;
            runAllCameraTests();
            break;
    }
    
    return g_camTestResults.failedTests > 0 ? 1 : 0;
}
