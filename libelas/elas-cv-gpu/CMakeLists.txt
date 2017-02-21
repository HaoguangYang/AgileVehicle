# Copyright: (c) 2017 AgileVehicle Team
# Author: Haoguang Yang
## Origin:
## Copyright: (C) 2011 RobotCub Consortium
## Author: Sean Ryan Fanello, Giulia Pasquale
## CopyPolicy: Released under the terms of the GNU GPL v2.0.
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

project(stereoVision)

############## LIBELAS ##############

if(UNIX)
    find_package(OpenMP)
endif()
find_package(CUDA)

if(CUDA_FOUND)
  message(STATUS "CUDA FOUND")
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch sm_50 -gencode arch=compute_50,code=sm_50)
  
  set(folder_source_elas src/elas/elas_gpu.cu
                         src/elas/descriptor.cpp
                         src/elas/filter.cpp
                         src/elas/matrix.cpp
                         src/elas/triangle.cpp)
  
else(if (OPENMP_FOUND))
  message(STATUS "OpenMP FOUND")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

  set(folder_source_elas src/elas/elas_omp.cpp
                         src/elas/descriptor.cpp
                         src/elas/filter.cpp
                         src/elas/matrix.cpp
                         src/elas/triangle.cpp)
else()
  message(STATUS "OpenMP NOT FOUND")
  set(folder_source_elas src/elas/elas.cpp
                         src/elas/descriptor.cpp
                         src/elas/filter.cpp
                         src/elas/matrix.cpp
                         src/elas/triangle.cpp)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")


####################################

set(folder_source src/camera.cpp
                  src/stereoCamera.cpp
                  src/disparityThread.cpp
                  src/opticalFlowThread.cpp
                  src/sceneFlow.cpp
                  src/elasWrapper.cpp)
if(CUDA_FOUND)
  set(folder_header include/camera.h
                  include/stereoCamera.h
                  include/disparityThread.h
                  include/opticalFlowThread.h
                  include/sceneFlow.h
                  include/elasWrapper.h
                  include/elas/elas_gpu.h
                  include/elas/descriptor.h
                  include/elas/image.h
                  include/elas/filter.h
                  include/elas/timer.h
                  include/elas/matrix.h
                  include/elas/triangle.h)
else()
  set(folder_header include/camera.h
                  include/stereoCamera.h
                  include/disparityThread.h
                  include/opticalFlowThread.h
                  include/sceneFlow.h
                  include/elasWrapper.h
                  include/elas/elas.h
                  include/elas/descriptor.h
                  include/elas/image.h
                  include/elas/filter.h
                  include/elas/timer.h
                  include/elas/matrix.h
                  include/elas/triangle.h)

#SIFT_GPU
#if(USE_SIFT_GPU)
#    list(APPEND folder_source src/utils.cpp)
#    list(APPEND folder_header include/iCub/stereoVision/utils.h)
#endif()

if(OpenCV_VERSION_MAJOR GREATER 2)
  add_definitions(-DOPENCV_GREATER_2)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/elas)

source_group("Source Files" FILES ${folder_source})
source_group("ELAS Source Files" FILES ${folder_source_elas})
source_group("Header Files" FILES ${folder_header})

include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

#if(USE_SIFT_GPU)
#    include_directories(${SIFTGPU_INCLUDE_DIRS})
#    add_definitions(-DUSING_GPU)
#endif()

add_library(${PROJECT_NAME} ${folder_source} ${folder_source_elas} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} iKin ${YARP_LIBRARIES})

#if(USE_SIFT_GPU)
#    target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
#endif()

if(OPENMP_FOUND)
   target_link_libraries(${PROJECT_NAME} ${OpenMP_C_FLAGS} ${OpenMP_CXX_FLAGS})
endif()

#if(USE_SIFT_GPU)
#   icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/iCub/stereoVision/elas
#                                              EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${SIFTGPU_INCLUDE_DIRS}
#                                              DESTINATION include/iCub/${PROJECT_NAME}
#                                              FILES_WITH_PATH ${folder_header}
#                                              PATH_TO_EXCLUDE include/iCub/stereoVision)
#else()
   icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/elas
                                              EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES}
                                              DESTINATION include
                                              FILES_WITH_PATH ${folder_header}
                                              PATH_TO_EXCLUDE include)
#endif()
