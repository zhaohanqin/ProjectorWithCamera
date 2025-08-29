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
add_executable(projectorTest ${CMAKE_CURRENT_SOURCE_DIR}/common/ProjectorTest.cpp)

# 设置可执行文件的包含目录
target_include_directories(projectorTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/common 
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/include
    ${MVS_INCLUDES_DIR}
)

# 链接必要的库
target_link_libraries(projectorTest 
    projectorDlpcApi
    ${OpenCV_LIBRARIES}
    ${MVS_CAMERA_CONTROL_LIB}
)

# 设置C++标准
target_compile_features(projectorTest PRIVATE cxx_std_17)
if (MSVC)
    target_compile_options(projectorTest PRIVATE /utf-8)
endif()

# 创建第二个可执行文件：ProjectorWithCamera.cpp（投影仪与相机协作流程示例）
add_executable(projectorWithCreame ${CMAKE_CURRENT_SOURCE_DIR}/common/ProjectorWithCamera.cpp)

# 设置可执行文件的包含目录
target_include_directories(projectorWithCreame PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/common 
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/include
    ${MVS_INCLUDES_DIR}
)

# 链接必要的库
target_link_libraries(projectorWithCreame 
    projectorDlpcApi
    ${OpenCV_LIBRARIES}
    ${MVS_CAMERA_CONTROL_LIB}
)
# 创建第三个可执行文件：CameraTest.cpp（相机连接与拍摄测试）
add_executable(cameraTest ${CMAKE_CURRENT_SOURCE_DIR}/common/CameraTest.cpp)

target_include_directories(cameraTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/common 
    ${MVS_INCLUDES_DIR}
)

target_link_libraries(cameraTest 
    ${OpenCV_LIBRARIES}
    ${MVS_CAMERA_CONTROL_LIB}
)

target_compile_features(cameraTest PRIVATE cxx_std_17)
if (MSVC)
    target_compile_options(cameraTest PRIVATE /utf-8)
endif()

# 设置C++标准
target_compile_features(projectorWithCreame PRIVATE cxx_std_17)
if (MSVC)
    target_compile_options(projectorWithCreame PRIVATE /utf-8)
endif()

# ==================== 复制DLL文件 ====================
# 确保运行时能找到cyusbserial.dll
add_custom_command(TARGET projectorTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial/cyusbserial.dll
    $<TARGET_FILE_DIR:projectorTest>
    COMMENT "Copying cyusbserial.dll to output directory"
)

add_custom_command(TARGET projectorWithCreame POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial/cyusbserial.dll
    $<TARGET_FILE_DIR:projectorWithCreame>
    COMMENT "Copying cyusbserial.dll to output directory"
)

add_custom_command(TARGET cameraTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial/cyusbserial.dll
    $<TARGET_FILE_DIR:cameraTest>
    COMMENT "Copying cyusbserial.dll to output directory"
)

# ==================== 设置运行时DLL搜索路径 ====================
# 为可执行文件设置额外的DLL搜索路径
set_target_properties(projectorTest PROPERTIES
    VS_DEBUGGER_ENVIRONMENT "PATH=${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial;$ENV{PATH}"
)

set_target_properties(projectorWithCreame PROPERTIES
    VS_DEBUGGER_ENVIRONMENT "PATH=${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial;$ENV{PATH}"
)

set_target_properties(cameraTest PROPERTIES
    VS_DEBUGGER_ENVIRONMENT "PATH=${CMAKE_CURRENT_SOURCE_DIR}/module/projector_dlpc_api/third_party/cyusbserial;$ENV{PATH}"
)

# ==================== 设置默认启动项目 ====================
# 设置cameraTest为默认启动项目（可按需切换）
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT cameraTest)
