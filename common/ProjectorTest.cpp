/**
 * @file projectorFactory_no_gtest.cpp
 * @author Evans Liu (1369215984@qq.com)
 * @brief 投影仪工厂类的完整功能测试套件
 * @version 0.1
 * @date 2024-03-19
 *
 * @copyright Copyright (c) 2024
 *
 * 本文件包含投影仪控制库的完整功能测试，涵盖：
 * 1. 投影仪初始化、连接、断开连接测试
 * 2. 投影控制功能测试（开始、停止、暂停、恢复、步进）
 * 3. 图案数据加载和投影测试
 * 4. LED亮度控制测试
 * 5. 硬件同步和时序控制测试
 *
 * 注意：此版本移除了Google Test依赖，使用标准C++实现
 */

#include "projectorFactory.h"
#include "projector.h"
#include "projectorDlpc34xx.h"
#include "projectorDlpc34xxDual.h" // 投影仪工厂类头文件，提供投影仪实例创建功能
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <cmath>
#include <cstring>
#include <algorithm>

// Windows控制台编码设置，解决中文显示乱码问题
#ifdef _WIN32
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#endif

// ==================== 控制台编码设置函数 ====================
/**
 * @brief 设置控制台编码为UTF-8，解决中文显示乱码问题
 * @details 在Windows环境下，将控制台代码页设置为UTF-8，确保中文字符正确显示
 */
void setupConsoleEncoding() {
#ifdef _WIN32
    // 设置控制台代码页为UTF-8
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    
    // 输出BOM标记，帮助某些终端识别UTF-8编码
    std::cout << "\xEF\xBB\xBF";
    
    // 尝试设置控制台字体为支持中文的字体
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hConsole != INVALID_HANDLE_VALUE) {
        CONSOLE_FONT_INFOEX cfi;
        cfi.cbSize = sizeof(cfi);
        cfi.nFont = 0;
        cfi.dwFontSize.X = 0;
        cfi.dwFontSize.Y = 16;
        cfi.FontFamily = FF_DONTCARE;
        cfi.FontWeight = FW_NORMAL;
        wcscpy_s(cfi.FaceName, L"Consolas");
        SetCurrentConsoleFontEx(hConsole, FALSE, &cfi);
    }
#endif
}

// ==================== 语言配置 ====================
// 设置为true使用中文输出，false使用英文输出
const bool USE_CHINESE_OUTPUT = true;

// 根据语言设置返回相应的字符串
std::string getLocalizedString(const std::string& chinese, const std::string& english) {
    return USE_CHINESE_OUTPUT ? chinese : english;
}

 // ==================== 测试数据常量定义 ====================
 // 测试用的投影仪型号常量，对应不同的DLP芯片
const std::string testProjector4710 = "DLP4710";    // DLP4710芯片投影仪，支持双通道
const std::string testProjector3010 = "DLP3010";    // DLP3010芯片投影仪，单通道

// 测试用的图案数据文件路径，包含实际的投影图案文件
const std::string testData4710 = "images_Projector";  // DLP4710测试图案数据路径
const std::string testData3010 = "../../test/data/4_3010";  // DLP3010测试图案数据路径

// ==================== 测试结果统计 ====================
struct TestResults {
    int totalTests = 0;
    int passedTests = 0;
    int failedTests = 0;

    void printSummary() const {
        std::cout << "\n=== " << getLocalizedString("测试结果汇总", "Test Results Summary") << " ===" << std::endl;
        std::cout << getLocalizedString("总测试数", "Total Tests") << ": " << totalTests << std::endl;
        std::cout << getLocalizedString("通过测试", "Passed Tests") << ": " << passedTests << std::endl;
        std::cout << getLocalizedString("失败测试", "Failed Tests") << ": " << failedTests << std::endl;
        std::cout << getLocalizedString("成功率", "Success Rate") << ": " << (totalTests > 0 ? (passedTests * 100.0 / totalTests) : 0) << "%" << std::endl;
    }
};

TestResults testResults; //初始化测试的结果

// ==================== 测试辅助函数 ====================
/**
 * @brief 断言函数：验证布尔条件是否为真
 * @param condition 要验证的布尔条件
 * @param message 测试描述信息
 * @details 此函数用于测试框架中验证各种条件是否满足：
 *          - 如果条件为真，测试通过，显示"✓"标记
 *          - 如果条件为假，测试失败，显示"✗"标记
 *          - 自动更新测试结果统计（总测试数、通过数、失败数）
 *          - 输出测试结果到控制台，便于调试和结果查看
 */
void assertTrue(bool condition, const std::string& message) {
    testResults.totalTests++;
    if (condition) {
        testResults.passedTests++;
        std::cout << "[PASS] " << message << std::endl;
    }
    else {
        testResults.failedTests++;
        std::cout << "[FAIL] " << message << std::endl;
    }
}

/**
 * @brief 断言函数：验证指针是否不为空
 * @param ptr 要验证的指针
 * @param message 测试描述信息
 * @details 此函数用于测试框架中验证指针的有效性：
 *          - 如果指针不为nullptr，测试通过，显示"✓"标记
 *          - 如果指针为nullptr，测试失败，显示"✗"标记
 *          - 常用于验证对象创建、内存分配等操作是否成功
 *          - 自动更新测试结果统计并输出结果到控制台
 */
void assertNotNull(void* ptr, const std::string& message) {
    testResults.totalTests++;
    if (ptr != nullptr) {
        testResults.passedTests++;
        std::cout << "[PASS] " << message << std::endl;
    }
    else {
        testResults.failedTests++;
        std::cout << "[FAIL] " << message << std::endl;
    }
}

// ==================== 连接辅助函数 ====================
// 尝试连接并立即通过 isConnect() 二次校验，只有两者都成功才认为连接成功
static bool connectAndVerify(slmaster::device::Projector* projector) {
    if (projector == nullptr) {
        return false;
    }
    bool ok = projector->connect();
    bool connected = projector->isConnect();
    if (!ok || !connected) {
        std::cout << "连接失败：connect()=" << (ok ? "true" : "false")
                  << ", isConnect()=" << (connected ? "true" : "false") << std::endl;
        return false;
    }
    return true;
}

// ==================== 基础功能测试 ====================

/**
 * @brief 投影仪初始化测试
 * @details 验证投影仪工厂类能够正确创建投影仪实例
 */
void testProjectorInit() {
    std::cout << "\n--- 测试投影仪初始化 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 通过工厂获取DLP4710投影仪实例
    assertNotNull(projectorDlpcApi, "投影仪实例创建成功，不为空指针");
}

/**
 * @brief 投影仪信息获取测试
 * @details 验证能够正确获取投影仪的基本信息和状态
 */
void testProjectorGetInfo() {
    std::cout << "\n--- 测试投影仪信息获取 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    auto info = projectorDlpcApi->getInfo();                                // 调用getInfo()获取投影仪详细信息
    assertTrue(info.isFind_, "投影仪被成功找到，isFind_标志为true");
}

// ==================== 连接控制测试 ====================

/**
 * @brief 投影仪连接测试
 * @details 验证投影仪能够成功建立硬件连接
 */
void testProjectorConnect() {
    std::cout << "\n--- 测试投影仪连接 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = connectAndVerify(projectorDlpcApi);                     // 连接并二次校验
    assertTrue(isSucess, "连接操作成功（connect+isConnect均为true）");
    if (!isSucess) return;                                                  // 连接失败则跳过后续

    isSucess = projectorDlpcApi->disConnect();                             // 测试断开连接功能，确保资源正确释放
    assertTrue(isSucess, "断开连接操作成功");
}

/**
 * @brief 投影仪连接状态检查测试
 * @details 验证连接状态检查功能是否正常工作
 */
void testProjectorIsConnect() {
    std::cout << "\n--- 测试投影仪连接状态检查 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = connectAndVerify(projectorDlpcApi);                     // 建立连接并校验
    assertTrue(isSucess, "投影仪连接成功（双重校验）");
    if (!isSucess) return;

    projectorDlpcApi->disConnect();                                        // 断开连接，清理资源
}

/**
 * @brief 投影仪断开连接测试
 * @details 验证断开连接功能是否正常工作
 */
void testProjectorDisconnect() {
    std::cout << "\n--- 测试投影仪断开连接 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = connectAndVerify(projectorDlpcApi);                     // 建立与投影仪的连接并校验
    assertTrue(isSucess, "投影仪连接成功（双重校验）");
    if (!isSucess) return;

    assertTrue(projectorDlpcApi->disConnect(), "断开连接操作成功，返回值为true");
}

// ==================== 投影控制功能测试 ====================

/**
 * @brief 单次投影测试
 * @details 验证非连续投影模式是否正常工作
 */
void testProjectorOnceProject() {
    std::cout << "\n--- 测试单次投影 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = connectAndVerify(projectorDlpcApi);                     // 建立与投影仪的连接并校验
    assertTrue(isSucess, "投影仪连接成功（双重校验）");
    if (!isSucess) return;

    isSucess = projectorDlpcApi->project(false);                           // 调用project(false)开始单次投影（非连续模式）
    assertTrue(isSucess, "单次投影操作成功");

    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒，让投影仪完成投影操作

    projectorDlpcApi->stop();                                              // 调用stop()停止投影
    projectorDlpcApi->disConnect();                                        // 断开连接，释放资源
}

/**
 * @brief 连续投影测试
 * @details 验证连续投影模式是否正常工作
 *
 * 注意：此测试使用的是投影仪内部的默认图案，而非用户自定义图像
 * - 投影仪内部存储有预设的测试图案（如棋盘格、条纹等）
 * - 这些默认图案用于基本功能验证和硬件测试
 * - 如需投影自定义图像，需要先调用populatePatternTableData()加载图案数据
 */
void testProjectorContinueProject() {
    std::cout << "\n--- 测试连续投影 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = connectAndVerify(projectorDlpcApi);                     // 建立与投影仪的连接并校验
    assertTrue(isSucess, "投影仪连接成功（双重校验）");
    if (!isSucess) return;

    // 开始连续投影模式 - 投影仪将自动循环播放内部默认图案
    // 注意：这里没有调用populatePatternTableData()，所以投影的是投影仪内置的测试图案
    // 内置图案通常包括：棋盘格、条纹、渐变等用于硬件验证的标准图案
    isSucess = projectorDlpcApi->project(true);                            // 调用project(true)开始连续投影模式
    assertTrue(isSucess, "连续投影操作成功");

    // 等待3秒观察连续投影效果
    // 在这段时间内，投影仪会自动循环播放内置图案，无需额外控制
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));          // 等待3秒，观察连续投影效果

    projectorDlpcApi->stop();                                              // 调用stop()停止连续投影
    isSucess = projectorDlpcApi->disConnect();                             // 断开连接
    assertTrue(isSucess, "断开连接操作成功");
}

/**
 * @brief 投影暂停功能测试
 * @details 验证投影暂停功能是否正常工作
 */
void testProjectorPause() {
    std::cout << "\n--- 测试投影暂停 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = connectAndVerify(projectorDlpcApi);                     // 建立与投影仪的连接并校验
    assertTrue(isSucess, "投影仪连接成功（双重校验）");
    if (!isSucess) return;

    isSucess = projectorDlpcApi->project(true);                            // 开始连续投影模式
    assertTrue(isSucess, "连续投影开始成功");

    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒，让投影仪开始投影

    isSucess = projectorDlpcApi->pause();                                 // 调用pause()暂停投影
    assertTrue(isSucess, "暂停操作成功");

    projectorDlpcApi->stop();                                              // 停止投影
    isSucess = projectorDlpcApi->disConnect();                             // 断开连接
    assertTrue(isSucess, "断开连接操作成功");
}

/**
 * @brief 投影恢复功能测试
 * @details 验证投影恢复功能是否正常工作
 */
void testProjectorResume() {
    std::cout << "\n--- 测试投影恢复 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = projectorDlpcApi->connect();                            // 建立与投影仪的连接
    assertTrue(isSucess, "投影仪连接成功");

    isSucess = projectorDlpcApi->project(true);                            // 开始连续投影模式
    assertTrue(isSucess, "连续投影开始成功");

    isSucess = projectorDlpcApi->pause();                                 // 暂停投影
    assertTrue(isSucess, "投影暂停成功");

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));          // 等待2秒，模拟暂停状态

    isSucess = projectorDlpcApi->resume();                                // 调用resume()恢复投影
    assertTrue(isSucess, "恢复操作成功");

    projectorDlpcApi->stop();                                              // 停止投影
    isSucess = projectorDlpcApi->disConnect();                             // 断开连接
    assertTrue(isSucess, "断开连接操作成功");
}

/**
 * @brief 投影停止功能测试
 * @details 验证投影停止功能是否正常工作
 */
void testProjectorStop() {
    std::cout << "\n--- 测试投影停止 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = connectAndVerify(projectorDlpcApi);                     // 建立与投影仪的连接并校验
    assertTrue(isSucess, "投影仪连接成功（双重校验）");
    if (!isSucess) return;

    isSucess = projectorDlpcApi->project(true);                            // 开始连续投影模式
    assertTrue(isSucess, "连续投影开始成功");

    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒，让投影仪开始投影

    isSucess = projectorDlpcApi->stop();                                  // 调用stop()停止投影
    assertTrue(isSucess, "停止操作成功");

    isSucess = projectorDlpcApi->disConnect();                             // 断开连接
    assertTrue(isSucess, "断开连接操作成功");
}

// ==================== 图案数据管理测试 ====================

/**
 * @brief 图案表数据填充测试
 * @details 验证将图案数据加载到投影仪内部闪存的功能
 * 这是结构光投影的核心功能，用于3D重建、表面检测等应用
 */
void testProjectorPopulatePatternTableData() {
    std::cout << "\n--- 测试图案表数据填充 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = connectAndVerify(projectorDlpcApi);                     // 建立与投影仪的连接并校验
    assertTrue(isSucess, "连接成功（双重校验）");
    if (!isSucess) return;

    // 创建两个图案集，用于测试双通道投影功能
    // 注意：这里的(2)表示创建2个PatternOrderSet元素，每个元素代表一个图案集
    // - 如果想要3个图案集，可以改为(3)
    // - 如果想要1个图案集，可以改为(1)
    // - 数量取决于投影仪的硬件能力和应用需求
    // - DLP4710支持双通道投影，可以同时投影两组不同的图案
    std::vector<slmaster::device::PatternOrderSet> patternSets(2);

    // ===== 配置第一个图案集参数 =====
    patternSets[0].exposureTime_ = 4000;                                   // 曝光时间4000微秒，控制投影图案的显示时长
    patternSets[0].preExposureTime_ = 3000;                                // 预曝光时间3000微秒，投影仪准备时间
    patternSets[0].postExposureTime_ = 3000;                               // 后曝光时间3000微秒，投影仪稳定时间
    patternSets[0].illumination_ = slmaster::device::Blue;                 // 蓝色LED照明，用于单色光投影
    patternSets[0].invertPatterns_ = false;                                // 不反转图案，保持原始图案
    patternSets[0].isVertical_ = true;                                     // 垂直图案，用于垂直条纹投影
    patternSets[0].isOneBit_ = false;                                      // 非1位深度，支持灰度图案
    patternSets[0].patternArrayCounts_ = 1920;                             // 图案数组数量1920，对应投影仪分辨率

    // ===== 配置第二个图案集参数（与第一个类似） =====
    patternSets[1].exposureTime_ = 4000;                                   // 第二个图案集的曝光时间
    patternSets[1].preExposureTime_ = 3000;                                // 第二个图案集的预曝光时间
    patternSets[1].postExposureTime_ = 3000;                               // 第二个图案集的后曝光时间
    patternSets[1].illumination_ = slmaster::device::Blue;                 // 第二个图案集的LED照明
    patternSets[1].invertPatterns_ = false;                                // 第二个图案集不反转
    patternSets[1].isVertical_ = true;                                     // 第二个图案集为垂直图案
    patternSets[1].isOneBit_ = false;                                      // 第二个图案集非1位深度
    patternSets[1].patternArrayCounts_ = 1920;                             // 第二个图案集的数组数量

    // ===== 加载测试图案文件 =====
    std::vector<cv::String> imgsPaths;                                     // 存储图案文件路径的向量
    cv::glob(testData4710 + "/*.png", imgsPaths);                          // 使用OpenCV的glob函数获取测试数据目录下的所有PNG文件路径
    std::vector<cv::Mat> imgFirstSet;                                      // 第一组图案的图像数据
    std::vector<cv::Mat> imgSecondSet;                                     // 第二组图案的图像数据

    // 添加调试信息
    std::cout << "找到PNG文件数量: " << imgsPaths.size() << std::endl;
    std::cout << "测试数据路径: " << testData4710 << std::endl;

    // 分别加载两组图案，每组包含多个图像文件
    for (size_t i = 0; i < imgsPaths.size() / 2; ++i) {
        // 加载第一组图案：从0开始的图像文件
        cv::Mat img1 = cv::imread(testData4710 + "/I" + std::to_string(i + 1) + ".png", 0);
        if (!img1.empty()) {
            imgFirstSet.push_back(img1);
        }
        // 加载第二组图案：从5开始的图像文件，cv::imread的第二个参数0表示灰度图像
        cv::Mat img2 = cv::imread(testData4710 + "/I" + std::to_string(i + 5) + ".png", 0);
        if (!img2.empty()) {
            imgSecondSet.push_back(img2);
        }
    }

    // 显示加载结果
    std::cout << "第一组图案加载数量: " << imgFirstSet.size() << std::endl;
    std::cout << "第二组图案加载数量: " << imgSecondSet.size() << std::endl;

    patternSets[0].imgs_ = imgFirstSet;                                    // 将第一组图像数据设置到第一个图案集
    patternSets[1].imgs_ = imgSecondSet;                                    // 将第二组图像数据设置到第二个图案集

    // 验证图案数据加载成功
    assertTrue(!patternSets[0].imgs_.empty(), "第一组图案非空");
    assertTrue(!patternSets[1].imgs_.empty(), "第二组图案非空");

    // 将图案数据加载到投影仪内部闪存
    isSucess = projectorDlpcApi->populatePatternTableData(patternSets);
    assertTrue(isSucess, "图案数据加载成功");

    // 开始投影并等待5秒观察效果
    projectorDlpcApi->project(true);                                       // 开始连续投影模式
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));          // 等待5秒

    // 停止投影并断开连接
    projectorDlpcApi->stop();                                              // 停止投影
    projectorDlpcApi->disConnect();                                        // 断开连接
}

// ==================== 步进投影测试 ====================

/**
 * @brief 步进投影功能测试
 * @details 验证逐帧投影功能，这是相机与投影仪同步的关键功能
 * 步进模式允许精确控制每一帧的投影时机，实现与相机的精确同步
 */
void testProjectorStep() {
    std::cout << "\n--- 测试步进投影 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = projectorDlpcApi->connect();                            // 建立与投影仪的连接
    assertTrue(isSucess, "投影仪连接成功");

    isSucess = projectorDlpcApi->project(true);                            // 开始投影模式
    assertTrue(isSucess, "投影模式开始成功");

    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒，让投影仪进入投影状态

    // 执行多次步进操作，每次投影下一帧图案
    isSucess = projectorDlpcApi->step();                                  // 第一次步进：投影下一帧
    assertTrue(isSucess, "第一次步进成功");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒

    isSucess = projectorDlpcApi->step();                                  // 第二次步进：投影下一帧
    assertTrue(isSucess, "第二次步进成功");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒

    isSucess = projectorDlpcApi->step();                                  // 第三次步进：投影下一帧
    assertTrue(isSucess, "第三次步进成功");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒

    isSucess = projectorDlpcApi->step();                                  // 第四次步进：投影下一帧
    assertTrue(isSucess, "第四次步进成功");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒

    isSucess = projectorDlpcApi->stop();                                  // 停止投影
    assertTrue(isSucess, "投影停止成功");

    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒

    isSucess = projectorDlpcApi->disConnect();                             // 断开连接
    assertTrue(isSucess, "断开连接操作成功");
}

/**
 * @brief 步进投影功能测试（带自定义图案和相机采集）
 * @details 验证逐帧投影功能，同时加载用户自定义图案数据
 * 这是相机与投影仪同步的完整实现，用于3D重建、表面检测等实际应用
 *
 * 注意：此函数需要先加载自定义图案数据，然后进行步进投影和相机采集
 */
void testProjectorStepWithCustomPatterns() {
    std::cout << "\n--- 测试步进投影（自定义图案+相机采集） ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = projectorDlpcApi->connect();                            // 建立与投影仪的连接
    assertTrue(isSucess, "投影仪连接成功");

    // ===== 加载自定义图案数据 =====
    // 注意与约束（务必满足，否则populatePatternTableData可能失败或显示异常）：
    // 1) 图像分辨率：必须与目标投影仪的DMD分辨率完全一致。
    //    - 例如：DLP4710 常见分辨率为 1920x1080；DLP3010 常见分辨率为 1280x720。
    //    - 分辨率不匹配会导致图案被裁剪、拉伸，甚至被设备拒绝加载。
    // 2) 图像通道与位深：建议使用单通道8位灰度（CV_8UC1）。
    //    - 本示例使用 cv::imread(path, 0) 强制灰度读取；若 isOneBit_ = true，则需确保图像为二值图（或在加载前自行阈值化）。
    //    - 如果固件只接受1-bit图案（某些高速模式），应将 isOneBit_ 置为 true，并提供二值化图像。
    // 3) 图案数量与序列：总数受图案表容量与设备闪存容量限制，过多会失败或耗时极长。
    //    - 建议先用少量（例如 10~20 张）验证流程，再逐步增加。
    //    - 图案顺序即为投影顺序；下文按 0..N-1 命名保证顺序稳定。
    // 4) 命名与排序：当前代码按 0.bmp, 1.bmp, ... 顺序加载；请确保文件连续、无缺号。
    // 5) 方向与条纹数：isVertical_ 表示条纹方向；patternArrayCounts_ 表示条纹/相位单元数量等固件相关参数。
    //    - 对垂直条纹通常与图像宽度相关（如 1920）；水平条纹则与图像高度相关（如 1080）。
    //    - 该参数需与固件/算法设置匹配，否则投影节奏与图案可能不同步。
    // 6) 曝光/前后沿时间：exposureTime_ / preExposureTime_ / postExposureTime_ 必须落在固件允许范围内。
    //    - 若过小/过大可能被设备拒绝或出现时序异常。请参考具体DLPC芯片数据手册。
    // 7) 光源与极性：illumination_ 应与实际使用的LED通道匹配；invertPatterns_ 与算法期望的黑白极性一致。
    // 8) USB与加载时长：首次加载会向设备写入大量数据，耗时与USB带宽、图案总量相关，属正常现象。
    // 创建图案集，用于存储用户自定义的投影图案
    std::vector<slmaster::device::PatternOrderSet> patternSets(1);         // 创建1个图案集，可根据需要调整数量

    // 配置图案集参数
    patternSets[0].exposureTime_ = 4000;                                   // 曝光时间4000微秒
    patternSets[0].preExposureTime_ = 3000;                                // 预曝光时间3000微秒
    patternSets[0].postExposureTime_ = 3000;                               // 后曝光时间3000微秒
    patternSets[0].illumination_ = slmaster::device::Blue;                 // 蓝色LED照明
    patternSets[0].invertPatterns_ = false;                                // 不反转图案
    patternSets[0].isVertical_ = true;                                     // 垂直图案
    patternSets[0].isOneBit_ = false;                                      // 非1位深度，支持灰度
    patternSets[0].patternArrayCounts_ = 1920;                             // 图案数组数量1920

    // 加载用户自定义的图案文件
    std::vector<cv::String> imgsPaths;                                     // 存储图案文件路径
    cv::glob(testData4710 + "/*.png", imgsPaths);                          // 获取测试数据目录下的所有PNG文件路径
    std::vector<cv::Mat> customPatterns;                                   // 自定义图案的图像数据

    // 添加调试信息
    std::cout << "找到PNG文件数量: " << imgsPaths.size() << std::endl;
    std::cout << "测试数据路径: " << testData4710 << std::endl;

    // 加载所有图案文件
    for (size_t i = 0; i < imgsPaths.size(); ++i) {
        // 加载图案文件，cv::imread的第二个参数0表示灰度图像
        cv::Mat img = cv::imread(imgsPaths[i], 0);
        if (!img.empty()) {
            customPatterns.push_back(img);
        }
    }

    // 显示加载结果
    std::cout << "自定义图案加载数量: " << customPatterns.size() << std::endl;

    patternSets[0].imgs_ = customPatterns;                                 // 将自定义图案设置到图案集

    // 验证图案数据加载成功
    assertTrue(!patternSets[0].imgs_.empty(), "自定义图案加载成功，图案集非空");

    // 将图案数据加载到投影仪内部闪存
    // 作用：
    // - 按 patternSets 中的参数（曝光/时序/方向/位深/极性/条纹计数等）
    //   将 imgs_ 中的自定义图案写入设备，并生成可被 project()/step() 使用的“图案表”。
    // 成功后：
    // - 调用 project(true) 激活该图案序列；随后每次 step() 前进一帧，实现步进投影。
    isSucess = projectorDlpcApi->populatePatternTableData(patternSets);//将图案数据加载到投影仪内部闪存
    assertTrue(isSucess, "自定义图案数据加载到投影仪成功");

    // ===== 开始步进投影和相机采集 =====
    isSucess = projectorDlpcApi->project(true);                            // 开始投影模式
    assertTrue(isSucess, "投影模式开始成功");

    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒，让投影仪进入投影状态

    // 执行多次步进操作，每次投影下一帧图案并进行相机采集
    isSucess = projectorDlpcApi->step();                                  // 第一次步进：投影下一帧
    assertTrue(isSucess, "第一次步进成功");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待投影稳定

    // 相机采集位置1：采集第一帧投影图案
    // camera.captureImage("frame_1.bmp");                                // 相机采集当前投影的图案
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));       // 等待相机采集完成

    isSucess = projectorDlpcApi->step();                                  // 第二次步进：投影下一帧
    assertTrue(isSucess, "第二次步进成功");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待投影稳定

    // 相机采集位置2：采集第二帧投影图案
    // camera.captureImage("frame_2.bmp");                                // 相机采集当前投影的图案
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));       // 等待相机采集完成

    isSucess = projectorDlpcApi->step();                                  // 第三次步进：投影下一帧
    assertTrue(isSucess, "第三次步进成功");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待投影稳定

    // 相机采集位置3：采集第三帧投影图案
    // camera.captureImage("frame_3.bmp");                                // 相机采集当前投影的图案
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));       // 等待相机采集完成

    isSucess = projectorDlpcApi->step();                                  // 第四次步进：投影下一帧
    assertTrue(isSucess, "第四次步进成功");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待投影稳定

    // 相机采集位置4：采集第四帧投影图案
    // camera.captureImage("frame_4.bmp");                                // 相机采集当前投影的图案
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));       // 等待相机采集完成

    isSucess = projectorDlpcApi->stop();                                  // 停止投影
    assertTrue(isSucess, "投影停止成功");

    std::this_thread::sleep_for(std::chrono::milliseconds(200));           // 等待200毫秒

    isSucess = projectorDlpcApi->disConnect();                             // 断开连接
    assertTrue(isSucess, "断开连接操作成功");

    std::cout << "步进投影测试完成，已投影 " << customPatterns.size() << " 帧自定义图案" << std::endl;
}

// ==================== 生成条纹图像与自动测试 ====================

/**
 * @brief 生成N步相移条纹图像（先垂直N张，再水平N张）
 * @param width      图像宽度（需与DMD一致）
 * @param height     图像高度（需与DMD一致）
 * @param frequency  条纹频率/周期数（整幅图中的正弦周期数量，等价于generate_fringe_patterns.py的frequency）
 * @param intensity  条纹强度（振幅，范围建议[0,127]，最终灰度=offset+intensity*cos(...))
 * @param offset     亮度偏移（平均灰度，范围[0,255]）
 * @param noiseLevel 噪声水平（高斯噪声标准差，0表示无噪声）
 * @param steps      相移步数N（返回图像总数为2N）
 * @return 按顺序返回2N张CV_8UC1图像：先垂直相位[0..2π)的N张，接着水平相位[0..2π)的N张
 */
static std::vector<cv::Mat> generatePhaseShiftFringeImages(int width, int height,
    int frequency, int intensity, int offset, double noiseLevel, int steps) {
    std::vector<cv::Mat> result;
    if (width <= 0 || height <= 0 || frequency <= 0 || steps <= 0) {
        return result;
    }

    // 约束参数，防止越界
    if (intensity < 0) intensity = 0;
    if (intensity > 255) intensity = 255;
    if (offset < 0) offset = 0;
    if (offset > 255) offset = 255;

    result.reserve(static_cast<size_t>(steps * 2));

    const double twoPi = 2.0 * 3.14159265358979323846;
    const double stepPhase = twoPi / static_cast<double>(steps);

    // 生成垂直条纹（沿x方向相位变化）
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

    // 生成水平条纹（沿y方向相位变化）
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

/**
 * @brief 使用自动生成的四步相移条纹图像进行步进投影测试（不依赖磁盘文件）
 * 生成顺序：先4张垂直，再4张水平；
 * - patternSets[0] 配置垂直条纹参数与图像
 * - patternSets[1] 配置水平条纹参数与图像
 */
void testProjectorStepWithGeneratedFringes() {
    std::cout << "\n--- 测试步进投影（自动生成四步相移条纹） ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710);
    bool isSucess = projectorDlpcApi->connect();
    assertTrue(isSucess, "投影仪连接成功");

    // 设备分辨率：请根据实际设备设置（示例为DLP4710常见分辨率）
    const int deviceWidth = 1920;
    const int deviceHeight = 1080;

    // 生成参数（与 generate_fringe_patterns.py 对齐）
    const int steps = 4;          // 相移步数N，可根据需要调整
    const int frequency = 32;     // 条纹频率/周期数
    const int intensity = 100;    // 振幅
    const int offset = 128;       // 亮度偏移
    const double noise = 0.0;     // 噪声标准差

    auto imgs = generatePhaseShiftFringeImages(deviceWidth, deviceHeight,
        frequency, intensity, offset, noise, steps);

    assertTrue(static_cast<int>(imgs.size()) == steps * 2, "生成2N张相移条纹图像");
    for (const auto& m : imgs) {
        assertTrue(m.type() == CV_8UC1, "图像为单通道8位灰度");
        assertTrue(m.cols == deviceWidth && m.rows == deviceHeight, "图像分辨率与设备一致");
    }

    std::cout << "成功生成 " << imgs.size() << " 张条纹图像" << std::endl;

    std::vector<slmaster::device::PatternOrderSet> patternSets(2);

    // 垂直条纹配置（前N张）
    patternSets[0].exposureTime_ = 4000;
    patternSets[0].preExposureTime_ = 3000;
    patternSets[0].postExposureTime_ = 3000;
    patternSets[0].illumination_ = slmaster::device::Blue;
    patternSets[0].invertPatterns_ = false;
    patternSets[0].isVertical_ = true;      // 垂直
    patternSets[0].isOneBit_ = false;       // 灰度正弦
    patternSets[0].patternArrayCounts_ = deviceWidth; // 对垂直条纹通常与宽度相关
    patternSets[0].imgs_.assign(imgs.begin(), imgs.begin() + steps);

    // 水平条纹配置（后N张）
    patternSets[1].exposureTime_ = 4000;
    patternSets[1].preExposureTime_ = 3000;
    patternSets[1].postExposureTime_ = 3000;
    patternSets[1].illumination_ = slmaster::device::Blue;
    patternSets[1].invertPatterns_ = false;
    patternSets[1].isVertical_ = false;     // 水平
    patternSets[1].isOneBit_ = false;       // 灰度正弦
    patternSets[1].patternArrayCounts_ = deviceHeight; // 水平条纹通常与高度相关
    patternSets[1].imgs_.assign(imgs.begin() + steps, imgs.end());

    std::cout << "垂直条纹图案数量: " << patternSets[0].imgs_.size() << std::endl;
    std::cout << "水平条纹图案数量: " << patternSets[1].imgs_.size() << std::endl;

    // 将图案数据加载到投影仪内部闪存
    std::cout << "开始加载图案数据到投影仪..." << std::endl;
    isSucess = projectorDlpcApi->populatePatternTableData(patternSets);
    assertTrue(isSucess, "生成条纹数据加载成功");
    
    if (!isSucess) {
        std::cout << "图案数据加载失败，无法继续测试" << std::endl;
        projectorDlpcApi->disConnect();
        return;
    }

    std::cout << "图案数据加载成功，开始设置LED亮度..." << std::endl;

    // 开启LED并设置亮度（确保投影仪有光源）
    isSucess = projectorDlpcApi->setLEDCurrent(0.8, 0.8, 0.8); // 设置80%亮度
    assertTrue(isSucess, "LED亮度设置成功");

    if (!isSucess) {
        std::cout << "LED亮度设置失败，但继续尝试投影..." << std::endl;
    }

    // 开始连续投影模式，确保投影仪能正常显示
    std::cout << "开始连续投影模式..." << std::endl;
    isSucess = projectorDlpcApi->project(true);
    assertTrue(isSucess, "连续投影模式开始成功");
    
    if (!isSucess) {
        std::cout << "投影模式启动失败，无法继续测试" << std::endl;
        projectorDlpcApi->disConnect();
        return;
    }

    // 等待投影仪稳定
    std::cout << "等待投影仪稳定..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 增加等待时间到2秒

    // 以固定次数步进，避免按集内计数重置导致的无限循环
    // 说明：DLPC 的 NumPatDisplayedFromPatSet 在跨 PatternSet 时会从 0 重新计数，
    // 仅依据该值与总帧数比较会导致死循环。这里采用确定次数的步进策略。
    const int totalFrames = steps * 2;
    std::cout << "开始步进投影，总共 " << totalFrames << " 帧..." << std::endl;
    
    for (int i = 0; i < totalFrames; ++i) {
        std::cout << "执行第 " << (i + 1) << " 次步进..." << std::endl;
        
        isSucess = projectorDlpcApi->step();
        assertTrue(isSucess, "步进第" + std::to_string(i + 1) + "步");
        
        if (!isSucess) {
            std::cout << "第 " << (i + 1) << " 次步进失败，停止测试" << std::endl;
            break;
        }
        
        // 等待投影仪完成当前图案的显示
        // 根据图案的曝光时间计算等待时间
        const bool isVertical = (i < steps);
        const int exposureTime = isVertical ? patternSets[0].exposureTime_ : patternSets[1].exposureTime_;
        const int preTime = isVertical ? patternSets[0].preExposureTime_ : patternSets[1].preExposureTime_;
        const int postTime = isVertical ? patternSets[0].postExposureTime_ : patternSets[1].postExposureTime_;
        const int totalTimeMs = (preTime + exposureTime + postTime) / 1000;
        
        // 增加等待时间，确保投影稳定
        // 对于DLP投影仪，需要足够的时间来稳定显示图案
        const int waitTimeMs = std::max(totalTimeMs + 500, 1500); // 最少1.5秒，确保投影稳定
        
        std::cout << "投影第" << (i + 1) << "帧图案（" << (isVertical ? "垂直" : "水平") << "条纹），等待" << waitTimeMs << "ms..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(waitTimeMs));
        
        std::cout << "第" << (i + 1) << "帧投影完成" << std::endl;
    }

    std::cout << "所有帧投影完成，停止投影..." << std::endl;
    isSucess = projectorDlpcApi->stop();
    assertTrue(isSucess, "投影停止成功");
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 等待投影仪完全停止
    
    isSucess = projectorDlpcApi->disConnect();
    assertTrue(isSucess, "断开连接操作成功");
    
    std::cout << "自动生成条纹测试完成" << std::endl;
}

// ==================== LED控制功能测试 ====================

/**
 * @brief LED电流获取和设置测试
 * @details 验证投影仪LED亮度控制功能
 * LED控制是结构光投影的重要参数，影响投影图案的对比度和相机采集质量
 */
void testProjectorGetSetLEDCurrent() {
    std::cout << "\n--- 测试LED电流获取和设置 ---" << std::endl;

    auto projectorFactory = slmaster::device::ProjectorFactory();           // 创建投影仪工厂实例
    auto projectorDlpcApi = projectorFactory.getProjector(testProjector4710); // 获取DLP4710投影仪实例
    bool isSucess = projectorDlpcApi->connect();                            // 建立与投影仪的连接
    assertTrue(isSucess, "投影仪连接成功");

    double red, green, blue;                                               // 声明RGB三色LED电流值变量
    isSucess = projectorDlpcApi->getLEDCurrent(red, green, blue);          // 获取当前LED电流值
    assertTrue(isSucess, "获取LED电流值成功");

    printf("projector's current light strength: red %f, green %f, blue %f \n", red, green, blue);
    printf("投影仪的当前亮度: red %f, green %f, blue %f \n", red, green, blue);

    // 设置LED电流值（0.95表示95%的最大亮度）
    // 这些值影响投影图案的亮度和对比度，对相机采集质量至关重要
    isSucess = projectorDlpcApi->setLEDCurrent(0.95, 0.95, 0.95);//影响到投影图案的亮度和对比度，对采集的质量至关重要
    assertTrue(isSucess, "设置LED电流值成功");

    // 再次获取LED电流值，验证设置结果是否正确
    isSucess = projectorDlpcApi->getLEDCurrent(red, green, blue);
    assertTrue(isSucess, "再次获取LED电流值成功");

    printf("after set light stength, projector's current light strength: red %f, green %f, blue %f \n", red, green, blue);
    printf("设置后的投影仪的当前亮度: red %f, green %f, blue %f \n", red, green, blue);

    isSucess = projectorDlpcApi->disConnect();                             // 断开连接
    assertTrue(isSucess, "断开连接操作成功");
}

// ==================== 主测试函数 ====================

/**
 * @brief 运行所有测试
 */
void runAllTests() {
    std::cout << getLocalizedString("开始运行投影仪功能测试...", "Starting projector functionality tests...") << std::endl;

    // 基础功能测试
    testProjectorInit();//测试投影仪的初始化是否成功
    testProjectorGetInfo();//测试投影仪的获取信息是否成功

    // 连接控制测试
    testProjectorConnect();//测试投影仪的连接是否成功
    testProjectorIsConnect();//测试投影仪的连接状态是否成功
    testProjectorDisconnect();//测试投影仪的断开连接是否成功

    // 投影控制测试
    testProjectorOnceProject();//测试投影仪的单次投影是否成功
    testProjectorContinueProject();//测试投影仪的连续投影是否成功
    testProjectorPause();//测试投影仪的暂停投影是否成功
    testProjectorResume();//测试投影仪的恢复投影是否成功
    testProjectorStop();//测试投影仪的停止投影是否成功

    // 图案数据管理测试
    testProjectorPopulatePatternTableData();//测试投影仪的图案数据管理是否成功

    // 步进投影测试
    testProjectorStep();//测试投影仪的步进投影是否成功

    // 步进投影测试（自定义图案+相机采集）
    testProjectorStepWithCustomPatterns();//测试投影仪的自定义图案步进投影和相机采集是否成功

    // 自动生成条纹测试
    testProjectorStepWithGeneratedFringes();//测试投影仪的自动生成条纹步进投影是否成功

    // LED控制测试
    testProjectorGetSetLEDCurrent();//测试投影仪的LED电流获取和设置是否成功

    // 打印测试结果
    testResults.printSummary();//打印测试结果
}

// ==================== 主函数 ====================

int main() {
    try {
        setupConsoleEncoding(); // 调用设置控制台编码的函数
        
        // 检测控制台编码是否支持中文
        std::cout << "Testing Chinese output: 测试中文输出" << std::endl;
        std::cout << "If you see garbled text above, the program will switch to English mode." << std::endl;
        std::cout << "如果上面显示乱码，程序将自动切换到英文模式。" << std::endl;
        
        // 等待用户确认
        std::cout << "\nPress Enter to continue... (按回车键继续...)" << std::endl;
        std::cin.get();
        
        runAllTests();
        return testResults.failedTests > 0 ? 1 : 0;  // 返回退出码，如果失败测试数大于0，则返回1，否则返回0
    }
    catch (const std::exception& e) {
        std::cerr << "测试过程中发生异常: " << e.what() << std::endl;
        return 1;
    }
}


