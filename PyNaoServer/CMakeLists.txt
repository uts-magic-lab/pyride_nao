# CMake file for PyNaoServer

cmake_minimum_required(VERSION 2.8)
project(PyNaoServer)
find_package(qibuild)

add_definitions( -DUSE_ENCRYPTION )

qi_create_lib( pynaoserver
  SRC main.cpp
      NaoProxyManager.cpp
      PyNaoServer.cpp
      PyNAOModule.cpp
  SHARED
)

qi_use_lib(pynaoserver ALCOMMON ALVISION ALAUDIO pyridecore tinyxml )
# qi_create_bin(...)

