cmake_minimum_required(VERSION 2.8.8)

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
endif()

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

#==================================================================
include(CMakeParseArguments)
include(GNUInstallDirs)
macro(icubcontrib_export_library target)
  cmake_parse_arguments(${target} "VERBOSE" "DESTINATION" "INTERNAL_INCLUDE_DIRS;EXTERNAL_INCLUDE_DIRS;DEPENDS;FILES;FILES_WITH_PATH;PATH_TO_EXCLUDE" ${ARGN})

  set(VERBOSE ${${target}_VERBOSE})
  if(VERBOSE)
    MESSAGE(STATUS "*** Arguments for ${target}")
    MESSAGE(STATUS "Internal directories: ${${target}_INTERNAL_INCLUDE_DIRS}")
    MESSAGE(STATUS "External directories: ${${target}_EXTERNAL_INCLUDE_DIRS}")
    MESSAGE(STATUS "Dependencies: ${${target}_DEPENDS}")
    MESSAGE(STATUS "Destination: ${${target}_DESTINATION}")
    MESSAGE(STATUS "Header files: ${${target}_FILES}")
    MESSAGE(STATUS "Header files for which we keep the relative path: ${${target}_FILES_WITH_PATH}")
    MESSAGE(STATUS "Part of the relative path to strip off: ${${target}_PATH_TO_EXCLUDE}")
    MESSAGE(STATUS "Option verbosity: ${${target}_VERBOSE}")
  endif()

  set(internal_includes ${${target}_INTERNAL_INCLUDE_DIRS})
  set(external_includes ${${target}_EXTERNAL_INCLUDE_DIRS})
  set(dependencies ${${target}_DEPENDS})
  set(files ${${target}_FILES})
  set(files_with_path ${${target}_FILES_WITH_PATH})
  set(path_to_exclude ${${target}_PATH_TO_EXCLUDE})
  set(destination ${${target}_DESTINATION})

  ##### Append target to global list.
  set_property(GLOBAL APPEND PROPERTY ICUBCONTRIB_TARGETS ${target})
  # Install/export rules
  install(TARGETS ${target} EXPORT icubcontrib-targets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development)
  if (MSVC) 
    install (FILES ${CMAKE_BINARY_DIR}/lib/Debug/${target}d.pdb 
                    DESTINATION ${CMAKE_INSTALL_LIBDIR} 
                    CONFIGURATIONS Debug 
                    COMPONENT Development
                    OPTIONAL)
  endif()
#TODO: move to "final" macro
 # install(EXPORT ${target}-targets DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME} FILE ${EXPORT_CONFIG_FILE} COMPONENT Development) #TODO check: use project_name, or ICUBcontrib?
# Export to build directory is disabled:
#  export(TARGETS ${target} FILE ${CMAKE_BINARY_DIR}/${target}-targets-build)
#important wrap ${dependencies} with "" to allows storing a list of dependencies
  set_target_properties(${target} PROPERTIES DEPENDS "${dependencies}")

##### Handle include directories
# Parsing dependencies
  if (dependencies)
    foreach (d ${dependencies})
        get_target_property(in_dirs ${d} INTERNAL_INCLUDE_DIRS)
        get_target_property(ext_dirs ${d} EXTERNAL_INCLUDE_DIRS)

        if (VERBOSE)
            message(STATUS "Getting from target ${d}:")
            message(STATUS "${in_dirs}")
            message(STATUS "${ext_dirs}")
        endif()

        if (in_dirs)
            set(internal_includes ${internal_includes} ${in_dirs})
        endif (in_dirs)

        if (ext_dirs)
            set(external_includes ${external_includes} ${ext_dirs})
        endif(ext_dirs)
    endforeach(d)
  endif(dependencies)
  ############################

  ################ Build unique variable with internal and external include directories
  ## Set corresponding target's properties
  set(include_dirs "")

  if (internal_includes)
    list(REMOVE_DUPLICATES internal_includes)
    set_target_properties(${target} PROPERTIES
                        INTERNAL_INCLUDE_DIRS
                        "${internal_includes}")
    if(VERBOSE)
        message(STATUS "Target ${target} exporting internal headers: ${internal_includes}")
    endif()
    list(APPEND include_dirs ${internal_includes})
  endif()

  if (external_includes)
    list(REMOVE_DUPLICATES external_includes)
    set_target_properties(${target} PROPERTIES
                        EXTERNAL_INCLUDE_DIRS
                        "${external_includes}")
    if(VERBOSE)
        message(STATUS "Target ${target} exporting external headers: ${external_includes}")
    endif()
    list(APPEND include_dirs ${external_includes})
  endif()

  if (include_dirs)
    list(REMOVE_DUPLICATES include_dirs)
    set_property(TARGET ${target} PROPERTY INCLUDE_DIRS  "${include_dirs}")
    if (VERBOSE)
        message(STATUS "Target ${target} exporting: ${include_dirs}")
    endif()
    set(${target}_INCLUDE_DIRS "${include_dirs}" CACHE STRING "include directories for target ${target}" FORCE)
  endif()

  ##############################################

  # Compile libraries using -fPIC to produce position independent code
  # For CMAKE_VERSION >= 2.8.10 this is handled in iCubOptions.cmake
  # using the CMAKE_POSITION_INDEPENDENT_CODE flag
#TODO insert option to enable PIC compilation
#  if(CMAKE_COMPILER_IS_GNUCXX AND NOT BUILD_SHARED_LIBS)
#    if(CMAKE_VERSION VERSION_EQUAL "2.8.9")
#      set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
#    elseif(CMAKE_VERSION VERSION_LESS "2.8.9")
#      set_target_properties(${target} PROPERTIES COMPILE_FLAGS -fPIC)
#    endif()
#  endif()



  #### Files export rules
  if (files AND destination)
    if (VERBOSE)
        message(STATUS "Target ${target} installing ${files} to ${destination}")
    endif()
    install(FILES ${files} DESTINATION ${destination} COMPONENT Development)

    set_target_properties(${target} PROPERTIES
                        HEADERFILES
                        "${files}")

    set_target_properties(${target} PROPERTIES
                        HEADERS_DESTINATION
                        ${destination})
  endif()

  #### Files export rules for files_with_path case
  if (files_with_path AND destination)
    if (VERBOSE)
        message(STATUS "Target ${target} installing ${files_with_path} to ${destination}")
    endif()
   

    if(path_to_exclude)
        # strip off the trailing slash
        string(REGEX REPLACE "/+$" "" path_to_exclude ${path_to_exclude})
    endif()

    foreach(cur_file  ${files_with_path})
        get_filename_component(file_rel_dir ${cur_file} PATH)
        if(path_to_exclude)
            string(REPLACE "${path_to_exclude}" "" file_rel_dir ${file_rel_dir})
        endif()
        install(FILES ${cur_file} DESTINATION ${destination}/${file_rel_dir} COMPONENT Development)
    endforeach()
    set_target_properties(${target} PROPERTIES
                        HEADERFILES_WITH_PATH
                        "${files_with_path}")

    set_target_properties(${target} PROPERTIES
                        HEADERS_DESTINATION
                        ${destination})
                        
  endif()

endmacro(icubcontrib_export_library)
#==================================================================

#if(USE_SIFT_GPU)
#   icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/iCub/stereoVision/elas
#                                              EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${SIFTGPU_INCLUDE_DIRS}
#                                              DESTINATION include/iCub/${PROJECT_NAME}
#                                              FILES_WITH_PATH ${folder_header}
#                                              PATH_TO_EXCLUDE include/iCub/stereoVision)
#else()
   icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/elas
                                              EXTERNAL_INCLUDE_DIRS ${OpenCV_LIBRARIES}
                                              DESTINATION include
                                              FILES_WITH_PATH ${folder_header}
                                              PATH_TO_EXCLUDE include)
#endif()

