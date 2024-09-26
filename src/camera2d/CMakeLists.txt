cmake_minimum_required(VERSION 3.8)
project(camera2d)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_executable(camera2d_node src/camera2d.cpp)
target_link_directories(camera2d_node PRIVATE $ENV{MVCAM_COMMON_RUNENV}/64)
target_link_libraries(camera2d_node
        libavutil.so
        libCLAllSerial_gcc485_v3_0.so
        libCLProtocol_gcc485_v3_0.so
        libCLSerCOM.so
        libCLSerHvc.so
        libFormatConversion.so
        libGCBase_gcc485_v3_0.so
        libGenCP_gcc485_v3_0.so
        liblog4cpp_gcc485_v3_0.so
        libLog_gcc485_v3_0.so
        libMediaProcess.so
        libMvCameraControl.so
        libMvCameraControlWrapper.so
        libMvCamLVision.so
        libMVFGControl.so
        libMVGigEVisionSDK.so
        libMVRender.so
        libMvSDKVersion.so
        libMvUsb3vTL.so
        libswscale.so
)
include_directories($ENV{MVCAM_SDK_PATH}/include)
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()
install(TARGETS camera2d_node
        DESTINATION lib/${PROJECT_NAME}
)
ament_auto_package()


