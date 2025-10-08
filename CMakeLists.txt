cmake_minimum_required(VERSION 3.20)

project(projector)

set(PROJECTOR_ROOT_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/common/projector.h
                          ${CMAKE_CURRENT_SOURCE_DIR}/common/projectorFactory.h
                          ${CMAKE_CURRENT_SOURCE_DIR}/common/typeDef.h)
# 移除源文件设置，因为现在都是可执行文件
# ==================== opencv文件配置 ====================
# 设置opencv文件路径
set(OpenCV_DIR "D:/opencv/opencv/build/x64/vc16/lib")

# ==================== MVS库文件配置 ====================
# 设置MVS库文件路径
set(MVS_INCLUDES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Includes)
set(MVS_LIBRARIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libraries/win64)

# 查找MVS库文件
find_library(MVS_CAMERA_CONTROL_LIB 
    NAMES MvCameraControl
    PATHS ${MVS_LIBRARIES_DIR}
    NO_DEFAULT_PATH
)

if(NOT MVS_CAMERA_CONTROL_LIB)
    message(WARNING "MvCameraControl library not found in ${MVS_LIBRARIES_DIR}")
endif()

find_package(OpenCV REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api)

# ==================== 库文件配置 ====================
# 只包含头文件，不包含源文件，因为源文件都是可执行文件
if(BUILD_SLMASTER_SHARED)
    add_library(projector SHARED)
    target_compile_definitions(projector PUBLIC -DBUILD_SHARED_LIBS)
    target_compile_definitions(projector PRIVATE -DDLL_EXPORTS)
else()
    add_library(projector)
endif()

# 只添加头文件，不添加源文件
target_sources(projector PUBLIC ${PROJECTOR_ROOT_HEADER})

target_include_directories(projector PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/common 
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/include
    ${MVS_INCLUDES_DIR}
)

target_link_libraries(
    projector
    PUBLIC
    projectorDlpcApi
)

# ==================== 可执行文件配置 ====================
# 创建第一个可执行文件：ProjectorTest.cpp
add_executable(ProjectorTest ${CMAKE_CURRENT_SOURCE_DIR}/common/ProjectorTest.cpp)

# 设置可执行文件的包含目录
target_include_directories(ProjectorTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/common 
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/include
    ${MVS_INCLUDES_DIR}
)

# 链接必要的库
target_link_libraries(ProjectorTest 
    projectorDlpcApi
    ${OpenCV_LIBRARIES}
    ${MVS_CAMERA_CONTROL_LIB}
)

# 设置C++标准
target_compile_features(ProjectorTest PRIVATE cxx_std_17)
if (MSVC)
    target_compile_options(ProjectorTest PRIVATE /utf-8)
endif()

# 创建第二个可执行文件：ProjectorWithCamera.cpp（投影仪与相机协作流程示例）
add_executable(ProjectorWithCamera ${CMAKE_CURRENT_SOURCE_DIR}/common/ProjectorWithCamera.cpp)

# 设置可执行文件的包含目录
target_include_directories(ProjectorWithCamera PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/common 
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/include
    ${MVS_INCLUDES_DIR}
)

# 链接必要的库
target_link_libraries(ProjectorWithCamera 
    projectorDlpcApi
    ${OpenCV_LIBRARIES}
    ${MVS_CAMERA_CONTROL_LIB}
)
# 创建第三个可执行文件：CameraTest.cpp（相机连接与拍摄测试）
add_executable(CameraTest ${CMAKE_CURRENT_SOURCE_DIR}/common/CameraTest.cpp)

target_include_directories(CameraTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/common 
    ${MVS_INCLUDES_DIR}
)

target_link_libraries(CameraTest 
    ${OpenCV_LIBRARIES}
    ${MVS_CAMERA_CONTROL_LIB}
)

target_compile_features(CameraTest PRIVATE cxx_std_17)
if (MSVC)
    target_compile_options(CameraTest PRIVATE /utf-8)
endif()

# 创建第四个可执行文件：Projector_Crmera_test.cpp（全白投影+相机拍照保存）
add_executable(Projector_Crmera_test ${CMAKE_CURRENT_SOURCE_DIR}/common/Projector_Crmera_test.cpp)

target_include_directories(Projector_Crmera_test PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/common 
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/include
    ${MVS_INCLUDES_DIR}
)

target_link_libraries(Projector_Crmera_test 
    projectorDlpcApi
    ${OpenCV_LIBRARIES}
    ${MVS_CAMERA_CONTROL_LIB}
)

target_compile_features(Projector_Crmera_test PRIVATE cxx_std_17)
if (MSVC)
    target_compile_options(Projector_Crmera_test PRIVATE /utf-8)
endif()

# 复制DLL
add_custom_command(TARGET Projector_Crmera_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial/cyusbserial.dll
    $<TARGET_FILE_DIR:Projector_Crmera_test>
    COMMENT "Copying cyusbserial.dll to output directory"
)

# VS 调试器 PATH 设置
set_target_properties(Projector_Crmera_test PROPERTIES
    VS_DEBUGGER_ENVIRONMENT "PATH=${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial;$ENV{PATH}"
)

# 设置C++标准
target_compile_features(ProjectorWithCamera PRIVATE cxx_std_17)
if (MSVC)
    target_compile_options(ProjectorWithCamera PRIVATE /utf-8)
endif()

# ==================== 复制DLL文件 ====================
# 确保运行时能找到cyusbserial.dll
add_custom_command(TARGET ProjectorTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial/cyusbserial.dll
    $<TARGET_FILE_DIR:ProjectorTest>
    COMMENT "Copying cyusbserial.dll to output directory"
)

add_custom_command(TARGET ProjectorWithCamera POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial/cyusbserial.dll
    $<TARGET_FILE_DIR:ProjectorWithCamera>
    COMMENT "Copying cyusbserial.dll to output directory"
)

add_custom_command(TARGET CameraTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial/cyusbserial.dll
    $<TARGET_FILE_DIR:CameraTest>
    COMMENT "Copying cyusbserial.dll to output directory"
)

# ==================== 设置运行时DLL搜索路径 ====================
# 为可执行文件设置额外的DLL搜索路径
set_target_properties(ProjectorTest PROPERTIES
    VS_DEBUGGER_ENVIRONMENT "PATH=${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial;$ENV{PATH}"
)

set_target_properties(ProjectorWithCamera PROPERTIES
    VS_DEBUGGER_ENVIRONMENT "PATH=${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial;$ENV{PATH}"
)

set_target_properties(CameraTest PROPERTIES
    VS_DEBUGGER_ENVIRONMENT "PATH=${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial;$ENV{PATH}"
)

# ==================== 设置默认启动项目 ====================
# 设置 Projector_Crmera_test 为默认启动项目（可按需切换）
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Projector_Crmera_test)
