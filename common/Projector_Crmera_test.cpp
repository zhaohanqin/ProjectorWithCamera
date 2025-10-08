// 全白投影 + 相机拍照保存
// 1) 使用投影仪加载并显示一张全白图（非连续步进+暂停，避免可见频闪）
// 2) 使用海康/MVS相机进行一次软触发采集，并将图像保存为PNG

#include "projectorFactory.h"
#include "projector.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

// MVS 工业相机 SDK
#include "MvCameraControl.h"

// 相机参数结构体与加载/应用
struct CameraParams {
    float exposureTimeUs = 20000.0f;  // 默认20ms曝光
    bool exposureAutoMode = false;
    float gainValue = 8.0f;           // 默认增益8
    bool gainAutoMode = false;
    float frameRate = 10.0f;          // 默认10fps
    int triggerDelayUs = 0;
    bool enableChunkData = false;
    bool printCurrentParams = true;
};

static const std::string CAMERA_PARAM_FILE = "camera_params.txt";

static bool loadCameraParams(CameraParams &params, const std::string &filename = CAMERA_PARAM_FILE) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "参数文件不存在: " << filename << "，将使用默认参数" << std::endl;
        return false;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);
        if (key == "exposureTimeUs") params.exposureTimeUs = std::stof(value);
        else if (key == "exposureAutoMode") params.exposureAutoMode = (value == "1");
        else if (key == "gainValue") params.gainValue = std::stof(value);
        else if (key == "gainAutoMode") params.gainAutoMode = (value == "1");
        else if (key == "frameRate") params.frameRate = std::stof(value);
        else if (key == "triggerDelayUs") params.triggerDelayUs = std::stoi(value);
        else if (key == "enableChunkData") params.enableChunkData = (value == "1");
        else if (key == "printCurrentParams") params.printCurrentParams = (value == "1");
    }
    std::cout << "参数已从文件加载: " << filename << std::endl;
    return true;
}

static bool configureCameraParams(void *handle, const CameraParams &params) {
    // 曝光
    MV_CC_SetEnumValue(handle, "ExposureAuto", params.exposureAutoMode ? 2 : 0);
    if (!params.exposureAutoMode && params.exposureTimeUs > 0) {
        MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTimeUs);
    }
    // 增益
    MV_CC_SetEnumValue(handle, "GainAuto", params.gainAutoMode ? 2 : 0);
    if (!params.gainAutoMode && params.gainValue > 0) {
        MV_CC_SetFloatValue(handle, "Gain", params.gainValue);
    }
    // 帧率
    if (params.frameRate > 0) {
        MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", params.frameRate);
    }
    // 触发延时
    if (params.triggerDelayUs > 0) {
        MV_CC_SetFloatValue(handle, "TriggerDelay", static_cast<float>(params.triggerDelayUs));
    }
    return true;
}

// 简单的软触发单帧采集并保存
static bool capture_one_image_and_save(const std::string &save_path, const CameraParams &params)
{
    void *handle = nullptr;

    MV_CC_DEVICE_INFO_LIST deviceList{};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK || deviceList.nDeviceNum == 0) {
        std::cerr << "未发现可用相机" << std::endl;
        return false;
    }

    MV_CC_DEVICE_INFO *pDevInfo = deviceList.pDeviceInfo[0];
    if (MV_CC_CreateHandle(&handle, pDevInfo) != MV_OK || !handle) {
        std::cerr << "创建相机句柄失败" << std::endl;
        return false;
    }

    if (MV_CC_OpenDevice(handle) != MV_OK) {
        std::cerr << "打开相机失败" << std::endl;
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 设置像素格式为Mono8
    MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");

    // 应用相机参数
    configureCameraParams(handle, params);

    // 触发模式=On，触发源=Software
    MV_CC_SetEnumValue(handle, "TriggerMode", 1);
    MV_CC_SetEnumValue(handle, "TriggerSource", 7);

    if (MV_CC_StartGrabbing(handle) != MV_OK) {
        std::cerr << "开始取流失败" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 稍作等待，确保设备就绪
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 发送软触发
    MV_CC_SetCommandValue(handle, "TriggerSoftware");

    // 获取图像缓存
    MV_FRAME_OUT frame{0};
    nRet = MV_CC_GetImageBuffer(handle, &frame, 5000);
    if (nRet != MV_OK || frame.pBufAddr == nullptr || frame.stFrameInfo.nWidth <= 0 || frame.stFrameInfo.nHeight <= 0) {
        std::cerr << "获取图像失败" << std::endl;
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 将图像保存为PNG
    cv::Mat img(frame.stFrameInfo.nHeight, frame.stFrameInfo.nWidth, CV_8UC1, frame.pBufAddr);
    bool ok = cv::imwrite(save_path, img);

    MV_CC_FreeImageBuffer(handle, &frame);
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

    if (!ok) {
        std::cerr << "保存图像失败: " << save_path << std::endl;
        return false;
    }

    std::cout << "图像已保存: " << save_path << std::endl;
    return true;
}

// 生成单张正弦条纹图（垂直/水平），CV_8UC1
static cv::Mat generateStripeImage(int width, int height, bool vertical, int frequency, int intensity, int offset)
{
    if (width <= 0 || height <= 0 || frequency <= 0) return cv::Mat();
    if (intensity < 0) intensity = 0; if (intensity > 255) intensity = 255;
    if (offset < 0) offset = 0; if (offset > 255) offset = 255;
    const double twoPi = 2.0 * 3.14159265358979323846;
    cv::Mat img(height, width, CV_8UC1);
    if (vertical) {
        for (int y = 0; y < height; ++y) {
            unsigned char *row = img.ptr<unsigned char>(y);
            for (int x = 0; x < width; ++x) {
                double t = static_cast<double>(x) / static_cast<double>(width);
                double g = static_cast<double>(offset) + static_cast<double>(intensity) * std::sin(twoPi * frequency * t);
                int gi = static_cast<int>(std::lround(g)); if (gi < 0) gi = 0; if (gi > 255) gi = 255;
                row[x] = static_cast<unsigned char>(gi);
            }
        }
    } else {
        for (int y = 0; y < height; ++y) {
            double t = static_cast<double>(y) / static_cast<double>(height);
            double g = static_cast<double>(offset) + static_cast<double>(intensity) * std::sin(twoPi * frequency * t);
            int gi = static_cast<int>(std::lround(g)); if (gi < 0) gi = 0; if (gi > 255) gi = 255;
            unsigned char v = static_cast<unsigned char>(gi);
            unsigned char *row = img.ptr<unsigned char>(y);
            std::memset(row, static_cast<int>(v), static_cast<size_t>(width));
        }
    }
    return img;
}

// 保存相机参数到文件
static bool saveCameraParams(const CameraParams &params, const std::string &filename = CAMERA_PARAM_FILE)
{
    try {
        std::ofstream f(filename, std::ios::trunc);
        if (!f.is_open()) return false;
        f << "# camera params\n";
        f << "exposureTimeUs=" << params.exposureTimeUs << "\n";
        f << "exposureAutoMode=" << (params.exposureAutoMode ? 1 : 0) << "\n";
        f << "gainValue=" << params.gainValue << "\n";
        f << "gainAutoMode=" << (params.gainAutoMode ? 1 : 0) << "\n";
        f << "frameRate=" << params.frameRate << "\n";
        f << "triggerDelayUs=" << params.triggerDelayUs << "\n";
        f << "enableChunkData=" << (params.enableChunkData ? 1 : 0) << "\n";
        f << "printCurrentParams=" << (params.printCurrentParams ? 1 : 0) << "\n";
        return true;
    } catch (...) {
        return false;
    }
}

// 新功能：投影单张图案（全白或条纹）后保持静止，并启动相机实时预览，允许现场调参并保存
// patternType: 0=全白, 1=垂直条纹, 2=水平条纹
// frequency: 条纹频率（仅对条纹有效）
static int run_projector_single_pattern_and_live_tuning(int patternType, int frequency = 15)
{
    using namespace slmaster::device;

    // 1) 准备投影图案
    ProjectorFactory factory;
    Projector *projector = factory.getProjector("DLP4710");
    if (!projector || !projector->connect()) {
        std::cerr << "投影仪获取/连接失败" << std::endl;
        return 1;
    }
    auto info = projector->getInfo();
    const int W = info.width_ > 0 ? info.width_ : 1920;
    const int H = info.height_ > 0 ? info.height_ : 1080;

    cv::Mat pattern;
    std::string patternName;
    if (patternType == 0) {
        // 全白图案
        pattern = cv::Mat(H, W, CV_8UC1, cv::Scalar(255));
        patternName = "全白图案";
    } else if (patternType == 1) {
        // 垂直条纹图案
        pattern = generateStripeImage(W, H, true, frequency, /*intensity*/100, /*offset*/128);
        if (pattern.empty()) pattern = cv::Mat(H, W, CV_8UC1, cv::Scalar(255));
        patternName = "垂直条纹图案(频率:" + std::to_string(frequency) + ")";
    } else {
        // 水平条纹图案
        pattern = generateStripeImage(W, H, false, frequency, /*intensity*/100, /*offset*/128);
        if (pattern.empty()) pattern = cv::Mat(H, W, CV_8UC1, cv::Scalar(255));
        patternName = "水平条纹图案(频率:" + std::to_string(frequency) + ")";
    }

    PatternOrderSet set;
    set.imgs_.push_back(pattern);
    set.patternArrayCounts_ = W;
    set.illumination_ = RGB;
    set.invertPatterns_ = false;
    set.isVertical_ = (patternType != 2); // 水平条纹时设为false
    set.isOneBit_ = false;
    set.exposureTime_ = 8000;
    set.preExposureTime_ = 5000;
    set.postExposureTime_ = 5000;
    std::vector<PatternOrderSet> patternSets{ set };

    std::cout << "设置LED电流..." << std::endl;
    if (!projector->setLEDCurrent(0.9, 0.9, 0.9)) {
        std::cerr << "LED电流设置失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    
    std::cout << "加载图案数据到投影仪..." << std::endl;
    if (!projector->populatePatternTableData(patternSets)) {
        std::cerr << "装载图案失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    
    // 执行投影仪稳定流程（完全按照ProjectorWithCamera.cpp的稳定方法）
    std::cout << "执行投影仪稳定流程..." << std::endl;
    projector->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    projector->disConnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    if (!projector->connect()) {
        std::cerr << "重新连接投影仪失败" << std::endl;
        return 1;
    }
    
    if (!projector->populatePatternTableData(patternSets)) {
        std::cerr << "重新装载图案失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    
    // 设置LED电流（与ProjectorWithCamera.cpp一致：等待500ms）
    std::cout << "设置LED电流..." << std::endl;
    projector->setLEDCurrent(0.9, 0.9, 0.9);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 启动连续投影模式（关键：等待2000ms让投影仪完全稳定）
    std::cout << "启动连续投影模式..." << std::endl;
    if (!projector->project(true)) {
        std::cerr << "启动连续投影模式失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 关键：充分等待
    
    // 停止并重新启动以确保稳定
    projector->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    if (!projector->project(true)) {
        std::cerr << "重新启动连续投影模式失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 关键修改：先暂停投影，让投影仪停在连续投影的当前帧（不要先step）
    std::cout << "暂停投影，准备步进控制..." << std::endl;
    projector->pause();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 等待pause生效
    
    // 验证LED电流设置（调试信息）
    double red, green, blue;
    if (projector->getLEDCurrent(red, green, blue)) {
        std::cout << "LED电流: R=" << red << " G=" << green << " B=" << blue << std::endl;
    }
    
    // 获取当前帧索引（如果需要定位，可以在这里step）
    // 对于单张图案，pause后就已经显示稳定，无需额外step
    std::cout << "投影仪已暂停在当前帧，准备实时预览..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    std::cout << "投影仪正在显示" << patternName << std::endl;
    std::cout << "图案尺寸: " << W << "x" << H << std::endl;

    // 2) 相机实时预览并允许调参
    CameraParams params{};
    loadCameraParams(params);

    void *handle = nullptr;
    MV_CC_DEVICE_INFO_LIST list{};
    if (MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list) != MV_OK || list.nDeviceNum == 0) {
        std::cerr << "未发现可用相机" << std::endl;
        projector->stop(); projector->disConnect();
        return 1;
    }
    MV_CC_DEVICE_INFO *pInfo = list.pDeviceInfo[0];
    if (MV_CC_CreateHandle(&handle, pInfo) != MV_OK || !handle) {
        std::cerr << "创建相机句柄失败" << std::endl;
        projector->stop(); projector->disConnect();
        return 1;
    }
    if (MV_CC_OpenDevice(handle) != MV_OK) {
        std::cerr << "打开相机失败" << std::endl;
        MV_CC_DestroyHandle(handle);
        projector->stop(); projector->disConnect();
        return 1;
    }

    MV_CC_SetEnumValueByString(handle, "PixelFormat", "Mono8");
    // 实时预览：关闭触发，连续采集
    MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    configureCameraParams(handle, params);
    if (MV_CC_StartGrabbing(handle) != MV_OK) {
        std::cerr << "开始连续取流失败" << std::endl;
        MV_CC_CloseDevice(handle); MV_CC_DestroyHandle(handle);
        projector->stop(); projector->disConnect();
        return 1;
    }

    std::cout << "实时预览：按键说明" << std::endl;
    std::cout << "  q/ESC: 退出  s: 保存参数到文件" << std::endl;
    std::cout << "  8: 曝光时间 +1000us" << std::endl;
    std::cout << "  2: 曝光时间 -1000us" << std::endl;
    std::cout << "  6: 增益 +0.5" << std::endl;
    std::cout << "  4: 增益 -0.5" << std::endl;
    std::cout << "\n投影仪状态：已暂停在" << patternName << "（稳定显示，无闪烁）" << std::endl;

    const int PREVIEW_W = 1080;
    const int PREVIEW_H = 720;
    cv::namedWindow("CameraPreview", cv::WINDOW_NORMAL);
    cv::resizeWindow("CameraPreview", PREVIEW_W, PREVIEW_H);
    bool running = true;
    while (running) {
        MV_FRAME_OUT fr{};
        int ret = MV_CC_GetImageBuffer(handle, &fr, 1000);
        if (ret == MV_OK && fr.pBufAddr && fr.stFrameInfo.nWidth > 0 && fr.stFrameInfo.nHeight > 0) {
            cv::Mat img(fr.stFrameInfo.nHeight, fr.stFrameInfo.nWidth, CV_8UC1, fr.pBufAddr);
            cv::Mat show; cv::cvtColor(img, show, cv::COLOR_GRAY2BGR);
            
            // 计算图像统计信息
            cv::Scalar meanVal = cv::mean(img);
            cv::Scalar meanStd;
            cv::meanStdDev(img, meanVal, meanStd);
            double minVal, maxVal;
            cv::minMaxLoc(img, &minVal, &maxVal);
            
            // 叠加相机参数信息 - 更大字体
            char buf1[300];
            std::snprintf(buf1, sizeof(buf1), "Camera: Exp=%.0fus  Gain=%.1f  FPS=%.1f", 
                          params.exposureTimeUs, params.gainValue, params.frameRate);
            cv::putText(show, buf1, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 255, 0}, 3);
            
            // 叠加图像质量统计信息 - 更大字体
            char buf2[300];
            std::snprintf(buf2, sizeof(buf2), "Image: Bright=%.1f  Contrast=%.1f  Range=[%.0f-%.0f]", 
                          meanVal[0], meanStd[0], minVal, maxVal);
            cv::putText(show, buf2, {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {255, 255, 0}, 3);
            
            // 添加图像质量判断提示 - 更大字体，英文提示
            std::string qualityText = "Quality: ";
            cv::Scalar qualityColor = {0, 255, 0}; // 默认绿色（良好）
            
            if (meanVal[0] < 30) {
                qualityText += "TOO DARK - Increase Exposure/Gain";
                qualityColor = {0, 0, 255}; // 红色
            } else if (meanVal[0] > 220) {
                qualityText += "TOO BRIGHT - Decrease Exposure/Gain";
                qualityColor = {0, 0, 255}; // 红色
            } else if (meanStd[0] < 8) {
                qualityText += "LOW CONTRAST - Adjust lighting";
                qualityColor = {0, 165, 255}; // 橙色
            } else {
                qualityText += "GOOD - Image quality is acceptable";
                qualityColor = {0, 255, 0}; // 绿色
            }
            
            cv::putText(show, qualityText, {20, 120}, cv::FONT_HERSHEY_SIMPLEX, 1.0, qualityColor, 3);
            
            // 添加详细的参数说明 - 更大字体
            cv::putText(show, "Bright: Average brightness (0-255, ideal: 50-200)", {20, 160}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {200, 200, 200}, 2);
            cv::putText(show, "Contrast: Standard deviation (>10 is good)", {20, 190}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {200, 200, 200}, 2);
            
            // 添加操作提示 - 更大字体
            cv::putText(show, "Controls: 8/2=Exposure  6/4=Gain  S=Save  Q=Exit", {20, 230}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 3);
            
            // 添加简单的亮度分布条形图（放大显示）
            int histHeight = 80;  // 从30增加到80
            int histWidth = 300;  // 从200增加到300
            int histX = show.cols - histWidth - 20;
            int histY = 20;
            
            // 创建简化的亮度分布显示（分成5个区间）
            std::vector<int> brightnessBins(5, 0);
            for (int y = 0; y < img.rows; y += 10) { // 采样，提高性能
                for (int x = 0; x < img.cols; x += 10) {
                    int pixelValue = img.at<uchar>(y, x);
                    int binIndex = std::min(4, pixelValue / 51); // 0-50, 51-102, 103-153, 154-204, 205-255
                    brightnessBins[binIndex]++;
                }
            }
            
            // 找到最大值用于归一化
            int maxBin = *std::max_element(brightnessBins.begin(), brightnessBins.end());
            if (maxBin > 0) {
                // 绘制背景
                cv::rectangle(show, cv::Point(histX-5, histY-5), cv::Point(histX+histWidth+5, histY+histHeight+5), {50, 50, 50}, -1);
                
                // 绘制各个区间的条形图
                for (int i = 0; i < 5; i++) {
                    int barHeight = (brightnessBins[i] * histHeight) / maxBin;
                    int barWidth = histWidth / 5;
                    int barX = histX + i * barWidth;
                    
                    // 根据亮度区间选择颜色
                    cv::Scalar barColor;
                    if (i == 0) barColor = {0, 0, 128};      // 很暗 - 深红
                    else if (i == 1) barColor = {0, 100, 255}; // 暗 - 橙色
                    else if (i == 2) barColor = {0, 255, 0};   // 正常 - 绿色
                    else if (i == 3) barColor = {255, 255, 0}; // 亮 - 青色
                    else barColor = {255, 0, 0};              // 很亮 - 蓝色
                    
                    cv::rectangle(show, cv::Point(barX, histY + histHeight - barHeight), 
                                 cv::Point(barX + barWidth - 2, histY + histHeight), barColor, -1);
                }
                
                // 添加标签 - 更大字体
                cv::putText(show, "Brightness Distribution", {histX, histY - 15}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);
                cv::putText(show, "Dark", {histX, histY + histHeight + 25}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {200, 200, 200}, 2);
                cv::putText(show, "Bright", {histX + histWidth - 60, histY + histHeight + 25}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {200, 200, 200}, 2);
                
                // 添加数值标签显示每个区间的像素数量
                for (int i = 0; i < 5; i++) {
                    int barWidth = histWidth / 5;
                    int barX = histX + i * barWidth;
                    int percentage = (brightnessBins[i] * 100) / (img.rows * img.cols / 100); // 计算百分比
                    std::string percentText = std::to_string(percentage) + "%";
                    cv::putText(show, percentText, {barX + barWidth/4, histY + histHeight + 45}, 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
                }
            }
            
            // 调整显示尺寸至 1080x720
            cv::resize(show, show, cv::Size(PREVIEW_W, PREVIEW_H), 0, 0, cv::INTER_AREA);
            cv::imshow("CameraPreview", show);
            MV_CC_FreeImageBuffer(handle, &fr);
        }

        int key = cv::waitKey(1);
        if (key < 0) continue;
        switch (key) {
            case 27: case 'q':
                running = false; break;
            case 's': {
                if (saveCameraParams(params)) std::cout << "参数已保存到 " << CAMERA_PARAM_FILE << std::endl;
                else std::cout << "参数保存失败" << std::endl;
                break;
            }
            case '8': {
                params.exposureTimeUs += 1000.0f;  // 增加曝光时间1ms
                if (params.exposureTimeUs > 100000.0f) params.exposureTimeUs = 100000.0f; // 最大100ms
                MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
                MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTimeUs);
                break;
            }
            case '2': {
                params.exposureTimeUs -= 1000.0f;  // 减小曝光时间1ms
                if (params.exposureTimeUs < 100.0f) params.exposureTimeUs = 100.0f; // 最小0.1ms
                MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
                MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTimeUs);
                break;
            }
            case '6': {
                params.gainValue += 0.5f;  // 增加增益0.5
                if (params.gainValue > 30.0f) params.gainValue = 30.0f; // 最大增益30
                MV_CC_SetEnumValue(handle, "GainAuto", 0);
                MV_CC_SetFloatValue(handle, "Gain", params.gainValue);
                break;
            }
            case '4': {
                params.gainValue -= 0.5f;  // 减小增益0.5
                if (params.gainValue < 0.0f) params.gainValue = 0.0f;   // 最小增益0
                MV_CC_SetEnumValue(handle, "GainAuto", 0);
                MV_CC_SetFloatValue(handle, "Gain", params.gainValue);
                break;
            }
            default: break;
        }
    }

    // 资源清理
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    projector->stop(); projector->disConnect();
    cv::destroyWindow("CameraPreview");
    return 0;
}

static int run_projector_white_capture()
{
    using namespace slmaster::device;

    // 连接投影仪
    ProjectorFactory factory;
    Projector *projector = factory.getProjector("DLP4710");
    if (!projector) {
        std::cerr << "获取投影仪实例失败" << std::endl;
        return 1;
    }
    if (!projector->connect()) {
        std::cerr << "连接投影仪失败" << std::endl;
        return 1;
    }

    // 获取分辨率
    auto info = projector->getInfo();
    const int W = info.width_ > 0 ? info.width_ : 1920;
    const int H = info.height_ > 0 ? info.height_ : 1080;

    // 生成一张与DMD分辨率一致的全白图
    cv::Mat white(H, W, CV_8UC1, cv::Scalar(255));

    // 组装图案集
    PatternOrderSet set;
    set.imgs_.push_back(white);
    set.patternArrayCounts_ = W;
    set.illumination_ = RGB;          // 使用三色LED合成白光
    set.invertPatterns_ = false;
    set.isVertical_ = true;
    set.isOneBit_ = false;            // 8位灰度更稳
    set.exposureTime_ = 8000;         // us
    set.preExposureTime_ = 5000;      // us
    set.postExposureTime_ = 5000;     // us

    std::vector<PatternOrderSet> patternSets{set};

    // 装载图案数据
    if (!projector->populatePatternTableData(patternSets)) {
        std::cerr << "装载全白图案失败" << std::endl;
        projector->disConnect();
        return 1;
    }

    // 执行投影仪稳定流程（按照ProjectorWithCamera.cpp的方法）
    std::cout << "执行投影仪稳定流程..." << std::endl;
    projector->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    projector->disConnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    if (!projector->connect()) {
        std::cerr << "重新连接投影仪失败" << std::endl;
        return 1;
    }
    
    if (!projector->populatePatternTableData(patternSets)) {
        std::cerr << "重新装载图案失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    
    // 设置LED电流
    projector->setLEDCurrent(0.95, 0.95, 0.95);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 启动连续投影模式（关键：先连续投影让投影仪稳定）
    std::cout << "启动连续投影模式..." << std::endl;
    if (!projector->project(true)) {
        std::cerr << "启动连续投影失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    projector->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    if (!projector->project(true)) {
        std::cerr << "重新启动连续投影失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 暂停投影（关键：先暂停，不要先step）
    std::cout << "暂停投影..." << std::endl;
    projector->pause();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 相机拍照并保存
    std::filesystem::create_directories("images");
    std::string outPath = (std::filesystem::path("images") / "white_capture.png").string();
    // 读取并应用相机参数
    CameraParams camParams{};
    loadCameraParams(camParams);
    bool capOK = capture_one_image_and_save(outPath, camParams);

    // 清理投影仪
    projector->stop();
    projector->disConnect();

    std::cout << (capOK ? "全白投影+拍摄完成" : "拍摄失败") << std::endl;
    return capOK ? 0 : 1;
}

int main()
{
#if defined(_WIN32)
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    
    // 禁用OpenCV的详细日志输出
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
    std::cout << "\n请选择要运行的功能:" << std::endl;
    std::cout << "  1) 全白投影+单次拍照保存" << std::endl;
    std::cout << "  2) 投影单张(全白)并启动相机实时预览可调参" << std::endl;
    std::cout << "  3) 投影单张(垂直条纹)并启动相机实时预览可调参" << std::endl;
    std::cout << "  4) 投影单张(水平条纹)并启动相机实时预览可调参" << std::endl;
    std::cout << "请输入序号并回车: ";
    int choice = 0;
    if (!(std::cin >> choice)) {
        std::cerr << "输入无效" << std::endl;
        return 1;
    }

    int frequency = 15; // 默认频率
    if (choice == 3 || choice == 4) {
        std::cout << "请输入条纹频率 (默认15): ";
        int inputFreq;
        if (std::cin >> inputFreq && inputFreq > 0 && inputFreq <= 100) {
            frequency = inputFreq;
        } else {
            std::cout << "使用默认频率: " << frequency << std::endl;
            std::cin.clear();
            std::cin.ignore(10000, '\n');
        }
    }

    int ret = 1;
    switch (choice) {
        case 1:
            ret = run_projector_white_capture();
            break;
        case 2:
            ret = run_projector_single_pattern_and_live_tuning(0); // 全白
            break;
        case 3:
            ret = run_projector_single_pattern_and_live_tuning(1, frequency); // 垂直条纹
            break;
        case 4:
            ret = run_projector_single_pattern_and_live_tuning(2, frequency); // 水平条纹
            break;
        default:
            std::cerr << "未知选项: " << choice << std::endl;
            ret = 1;
            break;
    }
    return ret;
}


