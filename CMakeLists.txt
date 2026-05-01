cmake_minimum_required(VERSION 3.16)

set(A750_CONTROL_CXX_COMPILER_OK OFF)

if(DEFINED CMAKE_CXX_COMPILER)
  set(A750_CONTROL_CXX_COMPILER_CANDIDATE "${CMAKE_CXX_COMPILER}")
elseif(DEFINED ENV{CXX})
  set(A750_CONTROL_CXX_COMPILER_CANDIDATE "$ENV{CXX}")
endif()

if(DEFINED A750_CONTROL_CXX_COMPILER_CANDIDATE AND
    A750_CONTROL_CXX_COMPILER_CANDIDATE MATCHES "(^|/)(x86_64-linux-gnu-)?g\\+\\+(-[0-9]+)?$")
  execute_process(
    COMMAND ${A750_CONTROL_CXX_COMPILER_CANDIDATE} -dumpfullversion -dumpversion
    RESULT_VARIABLE A750_CONTROL_CXX_VERSION_RESULT
    OUTPUT_VARIABLE A750_CONTROL_CXX_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(A750_CONTROL_CXX_VERSION_RESULT EQUAL 0 AND
      NOT A750_CONTROL_CXX_VERSION VERSION_LESS 14)
    set(A750_CONTROL_CXX_COMPILER_OK ON)
  endif()
endif()

if(NOT A750_CONTROL_CXX_COMPILER_OK)
  execute_process(
    COMMAND g++-14 --version
    RESULT_VARIABLE A750_CONTROL_GXX_14_RESULT
    OUTPUT_QUIET
    ERROR_QUIET
  )
  if(A750_CONTROL_GXX_14_RESULT EQUAL 0)
    set(CMAKE_CXX_COMPILER g++-14 CACHE STRING "C++ compiler" FORCE)
  endif()
endif()


get_filename_component(
  CURRENT_DIRNAME
  ${CMAKE_CURRENT_LIST_DIR}
  NAME
)

project(a750_control VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message(STATUS "a750_control C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "a750_control C++ compiler id: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} --version
  OUTPUT_VARIABLE A750_CONTROL_CXX_VERSION_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX REPLACE "\n.*" "" A750_CONTROL_CXX_VERSION "${A750_CONTROL_CXX_VERSION_OUTPUT}")
message(STATUS "a750_control C++ compiler version: ${A750_CONTROL_CXX_VERSION}")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 14)
  message(FATAL_ERROR
    "a750_control requires GCC 14 or newer, but CMake selected "
    "${CMAKE_CXX_COMPILER} (${CMAKE_CXX_COMPILER_VERSION}). "
    "Remove the existing build directory or configure with CXX=g++-14."
  )
endif()

set(A750_CONTROL_IS_TOP_LEVEL OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(A750_CONTROL_IS_TOP_LEVEL ON)
endif()

set(A750_CONTROL_EXCLUDE_FROM_ALL_DEFAULT OFF)
if(NOT A750_CONTROL_IS_TOP_LEVEL)
  set(A750_CONTROL_EXCLUDE_FROM_ALL_DEFAULT ON)
endif()

option(
  A750_CONTROL_EXCLUDE_FROM_ALL
  "Exclude a750_control targets from the default all target"
  ${A750_CONTROL_EXCLUDE_FROM_ALL_DEFAULT}
)

option(
  A750_CONTROL_BUILD_PYTHON
  "Build the a750_control Python extension"
  ${A750_CONTROL_IS_TOP_LEVEL}
)

if(A750_CONTROL_BUILD_PYTHON)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
endif()

include(cmake/CPM.cmake)

CPMAddPackage(
  NAME baselib
  GITHUB_REPOSITORY adob/baselib
  VERSION 0.0.0
  GIT_TAG 36f6c17ea5fcf481d9d6849566a980ca82b7d6e3
  EXCLUDE_FROM_ALL ${A750_CONTROL_EXCLUDE_FROM_ALL}
)

CPMAddPackage(
  NAME serialrpc
  GITHUB_REPOSITORY adob/serialrpc
  VERSION 0.0.0
  GIT_TAG bb8bcd4100c58f18c8a351ecc8f1570cd85dda35
  EXCLUDE_FROM_ALL ${A750_CONTROL_EXCLUDE_FROM_ALL}
)

CPMFindPackage(
  NAME protobuf
  VERSION 32.1.0
  GITHUB_REPOSITORY protocolbuffers/protobuf
  GIT_TAG v32.1
  EXCLUDE_FROM_ALL ${A750_CONTROL_EXCLUDE_FROM_ALL}
  OPTIONS
    "protobuf_BUILD_TESTS OFF"
    "protobuf_BUILD_CONFORMANCE OFF"
    "protobuf_BUILD_EXAMPLES OFF"
)

if(A750_CONTROL_BUILD_PYTHON)
  CPMFindPackage(
    NAME pybind11
    GITHUB_REPOSITORY pybind/pybind11
    VERSION 3.0.1
    GIT_TAG v3.0.1
    EXCLUDE_FROM_ALL ${A750_CONTROL_EXCLUDE_FROM_ALL}
  )
endif()

file(GLOB A750_CONTROL_PROTO_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_LIST_DIR}/proto/*.proto
)

set(A750_CONTROL_SERIALRPC_PROTO_DIR ${serialrpc_SOURCE_DIR})
set(A750_CONTROL_PROTOBUF_PROTO_DIR ${protobuf_SOURCE_DIR}/src)
set(A750_CONTROL_PROTO_DIR ${CMAKE_CURRENT_LIST_DIR}/proto)
set(A750_CONTROL_GENERATED_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
set(A750_CONTROL_GENERATED_PROTO_DIR ${A750_CONTROL_GENERATED_DIR}/proto)

set(A750_CONTROL_GENERATED_SOURCES)
set(A750_CONTROL_GENERATED_HEADERS)

file(MAKE_DIRECTORY ${A750_CONTROL_GENERATED_PROTO_DIR})

foreach(proto_file IN LISTS A750_CONTROL_PROTO_FILES)
  get_filename_component(proto_name ${proto_file} NAME_WE)

  set(proto_generated_sources
    ${A750_CONTROL_GENERATED_PROTO_DIR}/${proto_name}.pb_client.cc
    ${A750_CONTROL_GENERATED_PROTO_DIR}/${proto_name}.pb_msg.cc
    ${A750_CONTROL_GENERATED_PROTO_DIR}/${proto_name}.pb_server.cc
  )

  set(proto_generated_headers
    ${A750_CONTROL_GENERATED_PROTO_DIR}/${proto_name}.pb_client.h
    ${A750_CONTROL_GENERATED_PROTO_DIR}/${proto_name}.pb_msg.h
    ${A750_CONTROL_GENERATED_PROTO_DIR}/${proto_name}.pb_server.h
  )

  list(APPEND A750_CONTROL_GENERATED_SOURCES ${proto_generated_sources})
  list(APPEND A750_CONTROL_GENERATED_HEADERS ${proto_generated_headers})

  add_custom_command(
    OUTPUT
      ${proto_generated_sources}
      ${proto_generated_headers}
    COMMAND protobuf::protoc
      --plugin=protoc-gen-serialrpc=$<TARGET_FILE:serialrpcgen_plugin>
      --serialrpc_out=${A750_CONTROL_GENERATED_PROTO_DIR}
      -I ${A750_CONTROL_PROTO_DIR}
      -I ${A750_CONTROL_SERIALRPC_PROTO_DIR}
      -I ${A750_CONTROL_PROTOBUF_PROTO_DIR}
      ${proto_file}
    DEPENDS
      ${proto_file}
      ${A750_CONTROL_SERIALRPC_PROTO_DIR}/serialrpc.proto
      serialrpcgen_plugin
    VERBATIM
    COMMENT "Generating serialrpc sources from ${proto_name}.proto"
  )
endforeach()

file(GLOB SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

foreach(src_file IN LISTS SOURCES)
  if(src_file MATCHES ".*_(test|posix|experimental|benchmark|arduino)\\.cc$")
    list(REMOVE_ITEM SOURCES ${src_file})
  endif()

  if(src_file MATCHES "(^|[^A-Za-z0-9_])example([^A-Za-z0-9_]|$)")
    list(REMOVE_ITEM SOURCES ${src_file})
  endif()

  if(src_file MATCHES ".*\.pb_(client|msg|server)\.cc$")
    list(REMOVE_ITEM SOURCES ${src_file})
  endif()
endforeach()

list(APPEND SOURCES ${A750_CONTROL_GENERATED_SOURCES})

add_library(${PROJECT_NAME} STATIC
    ${SOURCES}
)

if(A750_CONTROL_EXCLUDE_FROM_ALL)
  set_property(TARGET ${PROJECT_NAME} PROPERTY EXCLUDE_FROM_ALL TRUE)
endif()

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

get_filename_component(_parent_dir
  "${CMAKE_CURRENT_LIST_DIR}/.."
  REALPATH
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${_parent_dir}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
  baselib::baselib
  serialrpc::serialrpc
)

add_dependencies(${PROJECT_NAME} serialrpcgen_plugin)

if(A750_CONTROL_BUILD_PYTHON)
  pybind11_add_module(_native MODULE
    python/bindings.cc
  )

  if(A750_CONTROL_EXCLUDE_FROM_ALL)
    set_property(TARGET _native PROPERTY EXCLUDE_FROM_ALL TRUE)
  endif()

  target_link_libraries(_native PRIVATE
    Python3::Module
    serialrpc::serialrpc
    baselib::baselib
    ${PROJECT_NAME}::${PROJECT_NAME}
  )

  target_compile_features(_native PRIVATE cxx_std_23)

  install(TARGETS _native
    LIBRARY DESTINATION a750_control
    RUNTIME DESTINATION a750_control
  )
endif()
