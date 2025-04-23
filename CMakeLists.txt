# cmake needs this line

cmake_minimum_required(VERSION 3.9.1)
 
# Define project name
project(onnxrun2)

# Kinect DK相机
find_package(k4a REQUIRED)
# Find OpenCV
find_package(OpenCV REQUIRED)
#find_package(Threads REQUIRED)
#find_package(OpenMP REQUIRED)
# Add includes
include_directories( ${CMAKE_CURRENT_LIST_DIR} ) # 包含当前目录下我的头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(include)

# Enable C++11
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CUDA_STANDARD 11)   
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
 
set(ONNXRUNTIME_INCLUDE_DIRS /home/xue/onnxruntime-linux-x64-gpu-1.7.0/include) 
set(ONNXRUNTIME_LIBS /home/xue/onnxruntime-linux-x64-gpu-1.7.0/lib/libonnxruntime.so) 
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
include_directories(${ONNXRUNTIME_INCLUDE_DIRS})


# Link your application with other libraries
add_executable(
    ${PROJECT_NAME} 	
    main.cpp
    src/serial_port.cpp
    src/tower.cpp 
    src/yolo.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${ONNXRUNTIME_INCLUDE_DIRS})
target_link_libraries(onnxrun2 ${CUDA_LIBRARIES})
target_link_libraries(${PROJECT_NAME} k4a::k4a ${ONNXRUNTIME_LIBS} ${OpenCV_LIBS})

# cmake needs this line
