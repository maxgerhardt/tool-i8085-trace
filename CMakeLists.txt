cmake_minimum_required(VERSION 3.13)

project(i8085-trace C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(i8085-trace
    src/main.cpp
    src/io_runtime.cpp
    src/disk_emu.cpp
    src/gdb_stub.cpp
    src/i8085_exec.c
    src/i8085_state.c
    src/i8085_disasm.c
)

target_include_directories(i8085-trace PRIVATE include)

if(UNIX AND NOT APPLE)
    target_link_libraries(i8085-trace PRIVATE dl)
endif()

if(WIN32)
    # Winsock2 (sockets) used by the GDB RSP stub.
    target_link_libraries(i8085-trace PRIVATE ws2_32)
endif()

# MC6850 ACIA / 28C256 EEPROM I/O plugin (loaded at runtime via --io-plugin).
add_library(mc6850_28c256 MODULE plugins/mc6850_28c256.cpp)
target_include_directories(mc6850_28c256 PRIVATE include)
# Keep a bare name (mc6850_28c256.dll/.so/.dylib), no "lib" prefix.
set_target_properties(mc6850_28c256 PROPERTIES PREFIX "")
if(APPLE)
    set_target_properties(mc6850_28c256 PROPERTIES SUFFIX ".dylib")
endif()
if(WIN32)
    target_link_libraries(mc6850_28c256 PRIVATE ws2_32)
endif()
