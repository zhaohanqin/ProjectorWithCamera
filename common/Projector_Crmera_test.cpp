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
    float exposureTimeUs = -1.0f;
    bool exposureAutoMode = false;
    float gainValue = -1.0f;
    bool gainAutoMode = false;
    float frameRate = -1.0f;
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
// useWhitePattern: true=全白；false=生成垂直条纹
static int run_projector_single_pattern_and_live_tuning(bool useWhitePattern)
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
    if (useWhitePattern) {
        pattern = cv::Mat(H, W, CV_8UC1, cv::Scalar(255));
    } else {
        // 生成一张垂直条纹图
        pattern = generateStripeImage(W, H, true, /*frequency*/15, /*intensity*/100, /*offset*/128);
        if (pattern.empty()) pattern = cv::Mat(H, W, CV_8UC1, cv::Scalar(255));
    }

    PatternOrderSet set;
    set.imgs_.push_back(pattern);
    set.patternArrayCounts_ = W;
    set.illumination_ = RGB;
    set.invertPatterns_ = false;
    set.isVertical_ = true;
    set.isOneBit_ = false;
    set.exposureTime_ = 8000;
    set.preExposureTime_ = 5000;
    set.postExposureTime_ = 5000;
    std::vector<PatternOrderSet> patternSets{ set };

    projector->setLEDCurrent(0.9, 0.9, 0.9);
    if (!projector->populatePatternTableData(patternSets) || !projector->project(false)) {
        std::cerr << "装载图案或进入步进模式失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    if (!projector->step()) {
        std::cerr << "步进显示单张图案失败" << std::endl;
        projector->stop(); projector->disConnect();
        return 1;
    }
    // 等待稳定并暂停
    int wait_ms = (set.preExposureTime_ + set.exposureTime_ + set.postExposureTime_) / 1000 + 500;
    if (wait_ms < 1000) wait_ms = 1000;
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    projector->pause();

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
    std::cout << "  +/- : 曝光×1.1 / ÷1.1" << std::endl;
    std::cout << "  g/G : 增益 +0.5 / -0.5" << std::endl;
    std::cout << "  f/F : 帧率 +1 / -1" << std::endl;
    std::cout << "  t/T : 触发延时 +500us / -500us" << std::endl;

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
            // 叠加当前参数
            char buf[256];
            std::snprintf(buf, sizeof(buf), "Exp(us):%.0f  Gain:%.2f  FPS:%.1f  Delay(us):%d",
                          params.exposureTimeUs, params.gainValue, params.frameRate, params.triggerDelayUs);
            cv::putText(show, buf, {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
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
            case '+': case '=': {
                if (params.exposureTimeUs <= 0) params.exposureTimeUs = 10000.0f;
                params.exposureTimeUs += 5556.0f;
                MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
                MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTimeUs);
                break;
            }
            case '-': case '_': {
                if (params.exposureTimeUs <= 0) params.exposureTimeUs = 10000.0f;
                params.exposureTimeUs -= 5556.0f; if (params.exposureTimeUs < 100.0f) params.exposureTimeUs = 100.0f;
                MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
                MV_CC_SetFloatValue(handle, "ExposureTime", params.exposureTimeUs);
                break;
            }
            case 'g': {
                if (params.gainValue < 0) params.gainValue = 0.0f; params.gainValue += 1.0f;
                MV_CC_SetEnumValue(handle, "GainAuto", 0);
                MV_CC_SetFloatValue(handle, "Gain", params.gainValue);
                break;
            }
            case 'G': {
                if (params.gainValue < 0) params.gainValue = 0.0f; params.gainValue -= 1.0f; if (params.gainValue < 0) params.gainValue = 0.0f;
                MV_CC_SetEnumValue(handle, "GainAuto", 0);
                MV_CC_SetFloatValue(handle, "Gain", params.gainValue);
                break;
            }
            case 'f': {
                if (params.frameRate < 0) params.frameRate = 5.0f; params.frameRate += 1.0f;
                MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", params.frameRate);
                break;
            }
            case 'F': {
                if (params.frameRate < 0) params.frameRate = 5.0f; params.frameRate -= 1.0f; if (params.frameRate < 1.0f) params.frameRate = 1.0f;
                MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", params.frameRate);
                break;
            }
            case 't': {
                params.triggerDelayUs += 100;
                MV_CC_SetFloatValue(handle, "TriggerDelay", static_cast<float>(params.triggerDelayUs));
                break;
            }
            case 'T': {
                params.triggerDelayUs -= 100; if (params.triggerDelayUs < 0) params.triggerDelayUs = 0;
                MV_CC_SetFloatValue(handle, "TriggerDelay", static_cast<float>(params.triggerDelayUs));
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

    // 设置LED电流（亮白）
    projector->setLEDCurrent(0.95, 0.95, 0.95);

    // 加载并进入非连续步进模式
    if (!projector->populatePatternTableData(patternSets)) {
        std::cerr << "装载全白图案失败" << std::endl;
        projector->disConnect();
        return 1;
    }
    if (!projector->project(false)) { // 非连续
        std::cerr << "进入非连续步进模式失败" << std::endl;
        projector->disConnect();
        return 1;
    }

    // 步进一步显示白图，随后暂停以保持稳定
    if (!projector->step()) {
        std::cerr << "步进显示白图失败" << std::endl;
        projector->stop();
        projector->disConnect();
        return 1;
    }

    // 等待投影稳定
    int wait_ms = (set.preExposureTime_ + set.exposureTime_ + set.postExposureTime_) / 1000 + 500;
    if (wait_ms < 1000) wait_ms = 1000;
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

    // 暂停，确保停留不闪烁
    projector->pause();

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
    std::cout << "\n请选择要运行的功能:" << std::endl;
    std::cout << "  1) 全白投影+单次拍照保存" << std::endl;
    std::cout << "  2) 投影单张(全白)并启动相机实时预览可调参" << std::endl;
    std::cout << "  3) 投影单张(垂直条纹)并启动相机实时预览可调参" << std::endl;
    std::cout << "请输入序号并回车: ";
    int choice = 0;
    if (!(std::cin >> choice)) {
        std::cerr << "输入无效" << std::endl;
        return 1;
    }

    int ret = 1;
    switch (choice) {
        case 1:
            ret = run_projector_white_capture();
            break;
        case 2:
            ret = run_projector_single_pattern_and_live_tuning(true);
            break;
        case 3:
            ret = run_projector_single_pattern_and_live_tuning(false);
            break;
        default:
            std::cerr << "未知选项: " << choice << std::endl;
            ret = 1;
            break;
    }
    return ret;
}


