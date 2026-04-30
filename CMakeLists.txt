cmake_minimum_required(VERSION 3.16)

get_filename_component(
  CURRENT_DIRNAME
  ${CMAKE_CURRENT_LIST_DIR}
  NAME
)

project(a750_control VERSION 0.0.0 LANGUAGES CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

include(cmake/CPM.cmake)

CPMAddPackage(
  NAME baselib
  GITHUB_REPOSITORY adob/baselib
  VERSION 0.0.0
  GIT_TAG 6470a8516b529194700ccf7eabe503936488b205
)

CPMAddPackage(
  NAME serialrpc
  GITHUB_REPOSITORY adob/serialrpc
  VERSION 0.0.0
  GIT_TAG 4253bea8338762de53dab1778b53563f640c6b66
)

CPMFindPackage(
  NAME protobuf
  VERSION 32.1.0
  GITHUB_REPOSITORY protocolbuffers/protobuf
  GIT_TAG v32.1
)

CPMFindPackage(
  NAME pybind11
  GITHUB_REPOSITORY pybind/pybind11
  VERSION 3.0.1
  GIT_TAG v3.0.1
)

file(GLOB A750_CONTROL_PROTO_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_LIST_DIR}/proto/*.proto
)

set(A750_CONTROL_SERIALRPC_PROTO_DIR ${serialrpc_SOURCE_DIR})
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
      --serialrpc_out=${A750_CONTROL_GENERATED_DIR}/proto
      -I=${CMAKE_CURRENT_LIST_DIR}
      -I=${A750_CONTROL_SERIALRPC_PROTO_DIR}
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

pybind11_add_module(_native MODULE
  python/bindings.cc
)

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
