cmake_minimum_required(VERSION 3.16.3)

project(Kaleidoscope)

# Find LLVM
find_package(LLVM REQUIRED CONFIG)

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message(STATUS "Using LLVMConfig.cmake in ${LLVM_DIR}")

if (NOT ${LLVM_ENABLE_ASSERTIONS})
    message(WARNING "Using an LLVM build without assertions")
endif()

add_executable(Kaleidoscope src/main.cpp)

llvm_map_components_to_libnames(LLVM_LIBRARIES core)

target_include_directories(Kaleidoscope PRIVATE ${LLVM_INCLUDE_DIRS})
target_link_libraries(Kaleidoscope ${LLVM_LIBRARIES})
