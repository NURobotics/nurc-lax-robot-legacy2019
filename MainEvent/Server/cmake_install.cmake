# Install script for directory: /Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/simple-websocket-server" TYPE FILE FILES
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/asio_compatibility.hpp"
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/server_ws.hpp"
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/client_ws.hpp"
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/server_wss.hpp"
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/client_wss.hpp"
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/crypto.hpp"
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/utility.hpp"
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/status_code.hpp"
    "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/Simple-WebSocket-Server/mutex.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/tests/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/medetm/LGR/nurc-lax-robot/MainEvent/Server/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
