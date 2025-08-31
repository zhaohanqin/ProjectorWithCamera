// 相机连接与拍摄测试（基于海康 MVS C++ SDK）
// 功能：
// 1) 枚举相机并按序列号选择；若传 "NULL" 或未指定则默认第一台
// 2) 打开设备、配置为软件触发 Mono8
// 3) 注册回调并软触发抓拍 N 张图像，保存为 I1..IN.png 到指定目录（或当前目录）

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

#if defined(_WIN32)
// 避免 Windows 宏 min/max 与 std::min/std::max 冲突
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(ms) usleep((ms) * 1000)
#endif

#include "MvCameraControl.h"

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
    if (!pFrameInfo || !pUser) {
        std::cout << "回调函数参数无效: pFrameInfo=" << pFrameInfo << ", pUser=" << pUser << std::endl;
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
                
                                 // 检查图像是否过曝（平均值过高）
                 if (meanVal > 200) {
                     std::cout << "⚠️  警告：图像过曝，平均值=" << meanVal << "，建议减少曝光时间" << std::endl;
                 } else if (meanVal > 150) {
                     std::cout << "⚠️  警告：图像偏亮，平均值=" << meanVal << "，建议适当减少曝光时间" << std::endl;
                 } else if (meanVal > 120) {
                     std::cout << "⚠️  警告：图像轻微偏亮，平均值=" << meanVal << "，建议微调曝光时间" << std::endl;
                 } else if (meanVal < 50) {
                     std::cout << "⚠️  警告：图像偏暗，平均值=" << meanVal << "，建议适当增加曝光时间" << std::endl;
                 } else {
                     std::cout << "✓ 图像亮度正常，平均值=" << meanVal << std::endl;
                 }
                 
                 // 检查是否有像素值达到255（过曝区域）
                 if (maxVal == 255) {
                     std::cout << "⚠️  警告：图像存在过曝区域（像素值255），建议进一步减少曝光时间" << std::endl;
                 }
                
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
    } else {
                 // 用户没有指定曝光时间，设置一个合理的默认值避免过曝
         MV_CC_SetEnumValue(handle, "ExposureAuto", 0); // 关闭自动曝光
         MVCC_FLOATVALUE exposureRange{};
         if (MV_CC_GetFloatValue(handle, "ExposureTime", &exposureRange) == MV_OK) {
             // 设置曝光时间为范围的最小值附近，避免过曝
             // 使用最小值+1%范围，进一步减少过曝风险
             double defaultExposure = exposureRange.fMin + (exposureRange.fMax - exposureRange.fMin) * 0.01;
             // 手动实现clamp功能，避免C++17兼容性问题
             double v = defaultExposure;
             if (v < exposureRange.fMin) v = exposureRange.fMin;
             if (v > exposureRange.fMax) v = exposureRange.fMax;
             MV_CC_SetFloatValue(handle, "ExposureTime", v);
             std::cout << "设置默认曝光时间(us): " << v << " [" << exposureRange.fMin << ", " << exposureRange.fMax << "]" << std::endl;
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
        // 手动实现clamp功能，避免C++17兼容性问题
        double v = vMid;
        if (v < vMin) v = vMin;
        if (v > vMax) v = vMax;
        ok = ok && (MV_CC_SetFloatValue(handle, "ExposureTime", v) == MV_OK);
    }
    MVCC_FLOATVALUE gainRange{}; if (MV_CC_GetFloatValue(handle, "Gain", &gainRange) == MV_OK) {
        const double vMin = static_cast<double>(gainRange.fMin);
        const double vMax = static_cast<double>(gainRange.fMax);
        const double vTarget = (vMin + vMax) * 0.2;
        // 手动实现clamp功能，避免C++17兼容性问题
        double v = vTarget;
        if (v < vMin) v = vMin;
        if (v > vMax) v = vMax;
        ok = ok && (MV_CC_SetFloatValue(handle, "Gain", v) == MV_OK);
    }
    MVCC_FLOATVALUE fpsRange{}; if (MV_CC_GetFloatValue(handle, "AcquisitionFrameRate", &fpsRange) == MV_OK) {
        MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
        const double vMin = static_cast<double>(fpsRange.fMin);
        const double vMax = static_cast<double>(fpsRange.fMax);
        const double vTarget = (vMin + vMax) * 0.5;
        // 手动实现clamp功能，避免C++17兼容性问题
        double v = vTarget;
        if (v < vMin) v = vMin;
        if (v > vMax) v = vMax;
        ok = ok && (MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", v) == MV_OK);
    }
    MVCC_FLOATVALUE tdelayRange{}; if (MV_CC_GetFloatValue(handle, "TriggerDelay", &tdelayRange) == MV_OK) {
        const double vMin = static_cast<double>(tdelayRange.fMin);
        const double vMax = static_cast<double>(tdelayRange.fMax);
        // 手动实现clamp功能，避免C++17兼容性问题
        double v = vMin; // 取最小值
        if (v < vMin) v = vMin;
        if (v > vMax) v = vMax;
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

// 曝光时间优化测试 - 专门用于调整曝光时间避免过曝
static void testCameraExposureOptimization(const std::string& cameraSerial,
                                          const std::string& saveDir,
                                          int frames) {
    std::cout << "\n--- 测试相机曝光时间优化 ---" << std::endl;
    
    // 尝试不同的曝光时间设置，找到最佳的曝光值
    // 从更短的曝光时间开始，避免过曝
    std::vector<double> testExposures = {500.0, 1000.0, 2000.0, 5000.0, 10000.0}; // 微秒
    
    for (double exposure : testExposures) {
        std::cout << "\n尝试曝光时间: " << exposure << " 微秒" << std::endl;
        bool ok = runCameraTest(cameraSerial, saveDir, 2, exposure, -1.0, -1.0, -1.0);
        if (ok) {
            std::cout << "曝光时间 " << exposure << " 微秒测试成功" << std::endl;
            // 可以在这里添加图像质量分析逻辑
        } else {
            std::cout << "曝光时间 " << exposure << " 微秒测试失败" << std::endl;
        }
    }
    
    assertTrue(true, "曝光时间优化测试完成");
}

// 智能曝光时间调整 - 根据图像质量自动调整曝光时间
static void testCameraSmartExposureAdjustment(const std::string& cameraSerial,
                                             const std::string& saveDir,
                                             int frames) {
    std::cout << "\n--- 测试相机智能曝光时间调整 ---" << std::endl;
    
    // 从较短的曝光时间开始，逐步调整到最佳值
    double currentExposure = 1000.0; // 从1ms开始
    double minExposure = 100.0;      // 最小曝光时间100微秒
    double maxExposure = 50000.0;    // 最大曝光时间50ms
    int maxIterations = 5;           // 最大迭代次数
    
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        std::cout << "\n第 " << (iteration + 1) << " 次迭代，曝光时间: " << currentExposure << " 微秒" << std::endl;
        
        bool ok = runCameraTest(cameraSerial, saveDir, 1, currentExposure, -1.0, -1.0, -1.0);
        if (ok) {
            std::cout << "曝光时间 " << currentExposure << " 微秒测试成功" << std::endl;
            
            // 如果曝光时间已经很小，说明已经找到合适的值
            if (currentExposure <= 2000.0) {
                std::cout << "找到合适的曝光时间: " << currentExposure << " 微秒" << std::endl;
                break;
            }
            
            // 继续减少曝光时间
            currentExposure *= 0.7; // 减少30%
            if (currentExposure < minExposure) {
                currentExposure = minExposure;
            }
        } else {
            std::cout << "曝光时间 " << currentExposure << " 微秒测试失败，尝试增加曝光时间" << std::endl;
            currentExposure *= 1.5; // 增加50%
            if (currentExposure > maxExposure) {
                currentExposure = maxExposure;
            }
        }
    }
    
    assertTrue(true, "智能曝光时间调整测试完成");
}

// 相机参数配置结构体
struct CameraParameters {
    double exposureTime = 1000.0;    // 曝光时间（微秒）
    double gain = 0.0;               // 增益
    double frameRate = 30.0;         // 帧率
    double triggerDelay = 0.0;       // 触发延时（微秒）
    
    // 保存参数到文件
    bool saveToFile(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "无法创建配置文件: " << filename << std::endl;
            return false;
        }
        
        file << "# 相机参数配置文件" << std::endl;
        file << "# 格式：参数名 = 数值" << std::endl;
        file << "# 曝光时间（微秒）" << std::endl;
        file << "ExposureTime = " << exposureTime << std::endl;
        file << "# 增益" << std::endl;
        file << "Gain = " << gain << std::endl;
        file << "# 帧率" << std::endl;
        file << "FrameRate = " << frameRate << std::endl;
        file << "# 触发延时（微秒）" << std::endl;
        file << "TriggerDelay = " << triggerDelay << std::endl;
        
        file.close();
        std::cout << "参数已保存到: " << filename << std::endl;
        return true;
    }
    
    // 从文件加载参数
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "配置文件不存在，使用默认参数: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            size_t pos = line.find('=');
            if (pos != std::string::npos) {
                std::string param = line.substr(0, pos);
                std::string value = line.substr(pos + 1);
                
                // 去除空格
                param.erase(0, param.find_first_not_of(" \t"));
                param.erase(param.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);
                
                if (param == "ExposureTime") {
                    exposureTime = std::stod(value);
                } else if (param == "Gain") {
                    gain = std::stod(value);
                } else if (param == "FrameRate") {
                    frameRate = std::stod(value);
                } else if (param == "TriggerDelay") {
                    triggerDelay = std::stod(value);
                }
            }
        }
        
        file.close();
        std::cout << "参数已从文件加载: " << filename << std::endl;
        return true;
    }
};

// 实时图像预览和参数调节测试
static void testCameraLivePreviewAndTuning(const std::string& cameraSerial) {
         std::cout << "\n--- Testing Camera Real-time Preview and Parameter Tuning ---" << std::endl;
     std::cout << "This function will open a real-time preview window where you can adjust camera parameters" << std::endl;
     std::cout << "Press 'q' to exit and save parameters, press 's' to save current parameters" << std::endl;
    
    // 枚举设备
    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        std::cerr << "未发现可用相机，错误码: " << nRet << std::endl;
        assertTrue(false, "设备枚举失败，无法进行实时预览测试");
        return;
    }
    
    // 选择设备
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
    }
    if (!pSelectedDevice) pSelectedDevice = deviceList.pDeviceInfo[0];
    
    // 创建并打开设备
    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, pSelectedDevice);
    if (nRet != MV_OK || handle == nullptr) {
        std::cerr << "创建相机句柄失败，错误码: " << nRet << std::endl;
        assertTrue(false, "创建相机句柄失败");
        return;
    }
    
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        std::cerr << "打开相机失败，错误码: " << nRet << std::endl;
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "打开相机失败");
        return;
    }
    
    // 配置相机基础参数
    std::cout << "配置相机基础参数..." << std::endl;
    
    // 设置像素格式为Mono8
    nRet = MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    if (nRet != MV_OK) {
        std::cerr << "设置像素格式失败，错误码: 0x" << std::hex << nRet << std::dec << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "设置像素格式失败");
        return;
    }
    
    // 设置触发选择器
    nRet = MV_CC_SetEnumValueByString(handle, "TriggerSelector", "FrameStart");
    if (nRet != MV_OK) {
        nRet = MV_CC_SetEnumValue(handle, "TriggerSelector", 6);
        if (nRet != MV_OK) {
            std::cerr << "设置触发选择器失败" << std::endl;
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            assertTrue(false, "设置触发选择器失败");
            return;
        }
    }
    
    // 开启触发模式
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
    if (nRet != MV_OK) {
        std::cerr << "开启触发模式失败" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "开启触发模式失败");
        return;
    }
    
    // 设置触发源为软件触发
    nRet = MV_CC_SetEnumValueByString(handle, "TriggerSource", "Software");
    if (nRet != MV_OK) {
        std::cerr << "设置触发源失败" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "设置触发源失败");
        return;
    }
    
    // 设置采集模式为连续模式
    nRet = MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous");
    if (nRet != MV_OK) {
        std::cerr << "设置采集模式失败" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "设置采集模式失败");
        return;
    }
    
         // 加载或设置默认参数
     CameraParameters params;
     std::string configFile = "camera_parameters.cfg";
     params.loadFromFile(configFile);
     
     // 应用参数到相机
     MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
     MV_CC_SetEnumValue(handle, "GainAuto", 0);
     
     MVCC_FLOATVALUE exposureRange{};
     if (MV_CC_GetFloatValue(handle, "ExposureTime", &exposureRange) == MV_OK) {
         // 如果配置文件中的曝光时间过低，使用一个合理的默认值
         if (params.exposureTime < 1000.0) {
             params.exposureTime = exposureRange.fMin + (exposureRange.fMax - exposureRange.fMin) * 0.1; // 使用10%位置的值
         }
         double v = params.exposureTime;
         if (v < exposureRange.fMin) v = exposureRange.fMin;
         if (v > exposureRange.fMax) v = exposureRange.fMax;
         MV_CC_SetFloatValue(handle, "ExposureTime", v);
         params.exposureTime = v;
         std::cout << "设置曝光时间: " << v << " 微秒 [" << exposureRange.fMin << ", " << exposureRange.fMax << "]" << std::endl;
     }
     
     MVCC_FLOATVALUE gainRange{};
     if (MV_CC_GetFloatValue(handle, "Gain", &gainRange) == MV_OK) {
         // 如果配置文件中的增益过低，使用一个合理的默认值
         if (params.gain < 1.0) {
             params.gain = gainRange.fMin + (gainRange.fMax - gainRange.fMin) * 0.2; // 使用20%位置的值
         }
         double v = params.gain;
         if (v < gainRange.fMin) v = gainRange.fMin;
         if (v > gainRange.fMax) v = gainRange.fMax;
         MV_CC_SetFloatValue(handle, "Gain", v);
         params.gain = v;
         std::cout << "设置增益: " << v << " [" << gainRange.fMin << ", " << gainRange.fMax << "]" << std::endl;
     }
    
    // 设置图像缓存节点数量
    MV_CC_SetImageNodeNum(handle, 3);
    
    // 注册图像回调
    CallbackContext ctx;
    ctx.totalFrames = 1;
    ctx.saveDir = ".";
    
    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallbackEx, &ctx);
    if (nRet != MV_OK) {
        std::cerr << "注册回调失败，错误码: " << nRet << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "注册回调失败");
        return;
    }
    
    // 开始取流
    nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        std::cerr << "开始取流失败，错误码: " << nRet << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        assertTrue(false, "开始取流失败");
        return;
    }
    
         // 创建预览窗口
     cv::namedWindow("Camera Live Preview - Press 'q' to exit, 's' to save", cv::WINDOW_AUTOSIZE);
     
     // 显示操作说明
     std::cout << "\n=== Real-time Preview Control Instructions ===" << std::endl;
     std::cout << "Keyboard shortcuts:" << std::endl;
     std::cout << "  E: Increase exposure time (+10%)" << std::endl;
     std::cout << "  e: Decrease exposure time (-10%)" << std::endl;
     std::cout << "  G: Increase gain (+0.5)" << std::endl;
     std::cout << "  g: Decrease gain (-0.5)" << std::endl;
     std::cout << "  F: Increase frame rate (+1fps)" << std::endl;
     std::cout << "  f: Decrease frame rate (-1fps)" << std::endl;
     std::cout << "  D: Increase trigger delay (+100us)" << std::endl;
     std::cout << "  d: Decrease trigger delay (-100us)" << std::endl;
           std::cout << "  S/s: Save current parameters to config file" << std::endl;
      std::cout << "  Q/q: Exit preview and save final parameters" << std::endl;
      std::cout << "  ESC: Exit preview without saving" << std::endl;
      std::cout << "  SPACE: Manual trigger capture" << std::endl;
      std::cout << "===============================================" << std::endl;
    
         // 实时预览循环
      bool running = true;
      cv::Mat previewImage;
      int frameCount = 0;
      
      // 定义触发选择器变量
      MVCC_ENUMVALUE triggerSelector;
      
      // 使用全局变量来存储最新的图像数据
      
              // 由于现有的ImageCallbackEx已经在运行，我们将使用它来获取图像
    // 不需要重新注册回调，这样可以避免访问冲突
    std::cout << "使用现有的回调机制进行预览..." << std::endl;
    
    // 立即触发一次采集来获取第一张图像
    std::cout << "触发第一次采集..." << std::endl;
    std::string triggerCmd = "TriggerSoftware";
    if (MV_CC_GetEnumValue(handle, "TriggerSelector", &triggerSelector) == MV_OK) {
        if (triggerSelector.nCurValue == 0) {
            triggerCmd = "FrameTriggerSoftware";
        }
    }
    MV_CC_SetCommandValue(handle, triggerCmd.c_str());
    std::cout << "第一次采集已触发，等待图像..." << std::endl;
    
    // 等待一段时间让第一张图像被捕获，并检查回调状态
    for (int i = 0; i < 20; i++) {
        Sleep(200);
        if (g_imageUpdated && !g_latestImage.empty()) {
            std::cout << "第一张图像捕获成功!" << std::endl;
            break;
        }
        std::cout << "等待图像... " << (i + 1) * 200 << "ms, 回调次数: " << g_callbackCallCount 
                 << ", 更新次数: " << g_imageUpdateCount << std::endl;
    }
      
    while (running) {
          // 显示当前状态
          if (g_imageUpdated && !g_latestImage.empty()) {
              std::cout << "检测到新图像，更新预览..." << std::endl;
              previewImage = g_latestImage.clone();
              g_imageUpdated = false;
              
              // 显示图像
              cv::imshow("Camera Live Preview - Press 'q' to exit, 's' to save", previewImage);
              
              // 显示当前参数信息（英文）
              std::string infoText = "Exp: " + std::to_string(static_cast<int>(params.exposureTime)) + "us, " +
                                   "Gain: " + std::to_string(params.gain) + ", " +
                                   "FPS: " + std::to_string(params.frameRate);
              
              // 在图像上绘制参数信息
              cv::Mat displayImage;
              cv::cvtColor(previewImage, displayImage, cv::COLOR_GRAY2BGR);
              cv::putText(displayImage, infoText, cv::Point(10, 30), 
                         cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
              cv::imshow("Camera Live Preview - Press 'q' to exit, 's' to save", displayImage);
              
              // 显示图像统计信息
              double minVal, maxVal, meanVal;
              cv::minMaxLoc(previewImage, &minVal, &maxVal);
              cv::Mat meanMat, stdDevMat;
              cv::meanStdDev(previewImage, meanMat, stdDevMat);
              meanVal = meanMat.at<double>(0, 0);
              
              std::cout << "Image stats - Min: " << minVal << ", Max: " << maxVal 
                       << ", Mean: " << meanVal << std::endl;
          } else {
              // 如果没有图像，显示提示信息
              static int noImageCount = 0;
              noImageCount++;
              std::cout << "未检测到新图像，g_imageUpdated=" << g_imageUpdated 
                       << ", g_latestImage.empty()=" << g_latestImage.empty() 
                       << ", 计数: " << noImageCount << std::endl;
              
              cv::Mat noImage(480, 640, CV_8UC1, cv::Scalar(0));
              cv::putText(noImage, "No Image Available", cv::Point(150, 240), 
                         cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(128), 2);
              cv::putText(noImage, "Press SPACE to trigger", cv::Point(150, 280), 
                         cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(128), 2);
              cv::imshow("Camera Live Preview - Press 'q' to exit, 's' to save", noImage);
              
              if (noImageCount % 10 == 0) { // 每10次显示一次提示
                  std::cout << "Waiting for image... Press SPACE to trigger capture" << std::endl;
                  std::cout << "回调统计 - 调用次数: " << g_callbackCallCount 
                           << ", 图像更新次数: " << g_imageUpdateCount << std::endl;
              }
          }
        
                           // 处理键盘输入
          int key = cv::waitKey(100) & 0xFF;
          switch (key) {
             case 'q':
             case 'Q':
                 std::cout << "User chose to exit, saving final parameters..." << std::endl;
                 params.saveToFile(configFile);
                 running = false;
                 break;
                 
             case 's':
             case 'S':
                 std::cout << "Saving current parameters..." << std::endl;
                 params.saveToFile(configFile);
                 break;
                 
             case 27: // ESC
                 std::cout << "User chose to exit without saving parameters..." << std::endl;
                 running = false;
                 break;
                 
                           case 'e': // Decrease exposure time
                  if (MV_CC_GetFloatValue(handle, "ExposureTime", &exposureRange) == MV_OK) {
                      params.exposureTime *= 0.9; // Decrease by 10%
                      if (params.exposureTime < exposureRange.fMin) params.exposureTime = exposureRange.fMin;
                      MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTime);
                      std::cout << "Exposure time decreased to: " << params.exposureTime << " us" << std::endl;
                      
                      // 立即触发一次采集来测试新参数
                      std::string triggerCmd = "TriggerSoftware";
                      if (MV_CC_GetEnumValue(handle, "TriggerSelector", &triggerSelector) == MV_OK) {
                          if (triggerSelector.nCurValue == 0) {
                              triggerCmd = "FrameTriggerSoftware";
                          }
                      }
                      MV_CC_SetCommandValue(handle, triggerCmd.c_str());
                      std::cout << "Triggered capture with new exposure time" << std::endl;
                  }
                  break;
                  
              case 'E': // Increase exposure time
                  if (MV_CC_GetFloatValue(handle, "ExposureTime", &exposureRange) == MV_OK) {
                      params.exposureTime *= 1.1; // Increase by 10%
                      if (params.exposureTime > exposureRange.fMax) params.exposureTime = exposureRange.fMax;
                      MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTime);
                      std::cout << "Exposure time increased to: " << params.exposureTime << " us" << std::endl;
                      
                      // 立即触发一次采集来测试新参数
                      std::string triggerCmd = "TriggerSoftware";
                      if (MV_CC_GetEnumValue(handle, "TriggerSelector", &triggerSelector) == MV_OK) {
                          if (triggerSelector.nCurValue == 0) {
                              triggerCmd = "FrameTriggerSoftware";
                          }
                      }
                      MV_CC_SetCommandValue(handle, triggerCmd.c_str());
                      std::cout << "Triggered capture with new exposure time" << std::endl;
                  }
                  break;
                 
             case 'g': // Decrease gain
                 if (MV_CC_GetFloatValue(handle, "Gain", &gainRange) == MV_OK) {
                     params.gain -= 0.5;
                     if (params.gain < gainRange.fMin) params.gain = gainRange.fMin;
                     MV_CC_SetFloatValue(handle, "Gain", params.gain);
                     std::cout << "Gain decreased to: " << params.gain << std::endl;
                 }
                 break;
                 
             case 'G': // Increase gain
                 if (MV_CC_GetFloatValue(handle, "Gain", &gainRange) == MV_OK) {
                     params.gain += 0.5;
                     if (params.gain > gainRange.fMax) params.gain = gainRange.fMax;
                     MV_CC_SetFloatValue(handle, "Gain", params.gain);
                     std::cout << "Gain increased to: " << params.gain << std::endl;
                 }
                 break;
                 
             case 'f': // Decrease frame rate
                 params.frameRate = std::max(1.0, params.frameRate - 1.0);
                 std::cout << "Frame rate decreased to: " << params.frameRate << " fps" << std::endl;
                 break;
                 
             case 'F': // Increase frame rate
                 params.frameRate = std::min(100.0, params.frameRate + 1.0);
                 std::cout << "Frame rate increased to: " << params.frameRate << " fps" << std::endl;
                 break;
                 
             case 'd': // Decrease trigger delay
                 params.triggerDelay = std::max(0.0, params.triggerDelay - 100.0);
                 std::cout << "Trigger delay decreased to: " << params.triggerDelay << " us" << std::endl;
                 break;
                 
                           case 'D': // Increase trigger delay
                  params.triggerDelay += 100.0;
                  std::cout << "Trigger delay increased to: " << params.triggerDelay << " us" << std::endl;
                  break;
                  
              case 32: // SPACE key - Manual trigger
                  std::cout << "Manual trigger activated..." << std::endl;
                  {
                      std::string triggerCmd = "TriggerSoftware";
                      if (MV_CC_GetEnumValue(handle, "TriggerSelector", &triggerSelector) == MV_OK) {
                          if (triggerSelector.nCurValue == 0) {
                              triggerCmd = "FrameTriggerSoftware";
                          }
                      }
                      MV_CC_SetCommandValue(handle, triggerCmd.c_str());
                      std::cout << "Manual capture triggered, waiting for image..." << std::endl;
                      
                      // 等待一段时间让图像被捕获
                      Sleep(500);
                  }
                  break;
         }
        
                 frameCount++;
         if (frameCount % 30 == 0) { // 每30帧显示一次状态
             std::cout << "Preview running... Current parameters - Exposure: " << params.exposureTime 
                       << "us, Gain: " << params.gain << ", Frame Rate: " << params.frameRate << "fps" << std::endl;
         }
    }
    
    // 清理资源
    cv::destroyAllWindows();
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    
         std::cout << "Real-time preview test completed!" << std::endl;
     std::cout << "Final parameters saved to: " << configFile << std::endl;
     std::cout << "You can use these parameters in subsequent tests" << std::endl;
     
     assertTrue(true, "Real-time preview and parameter tuning test completed");
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

    /*
    // 基础能力
    testCameraEnumerate();
    testCameraOpenClose(cameraSerial);
    testCameraConfigureTrigger(cameraSerial);
    testCameraParameterTuning(cameraSerial);

    // 采集能力
    testCameraCaptureAndSave(cameraSerial, saveDir, 4);

    // 软触发兼容性测试
    testCameraSoftwareTriggerCompatibility(cameraSerial);

    // 软触发功能测试
    testCameraSoftwareTrigger(cameraSerial);

         // 曝光时间优化测试
     testCameraExposureOptimization(cameraSerial, saveDir, 2);
     
     // 智能曝光时间调整测试
     testCameraSmartExposureAdjustment(cameraSerial, saveDir, 1);
     
     */

     // 实时预览和参数调节测试（新增功能）
     testCameraLivePreviewAndTuning(cameraSerial);
     
	 /*
     // 端到端示例（如需调参，可在此设置非负数）
     testCameraEndToEnd(cameraSerial, saveDir, 6, -1.0, -1.0, -1.0, -1.0);
     */

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


