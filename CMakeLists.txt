cmake_minimum_required(VERSION 3.13)

project(i8085-trace C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(i8085-trace
    src/main.cpp
    src/io_runtime.cpp
    src/io_channel.cpp
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

# --- Runtime I/O plugins (loaded via --io-plugin) ---------------------------
# Helper to declare a runtime plugin module with a bare (no "lib") name.
function(add_i8085_plugin name)
    add_library(${name} MODULE plugins/${name}.cpp)
    target_include_directories(${name} PRIVATE include)
    set_target_properties(${name} PROPERTIES PREFIX "")
    if(APPLE)
        set_target_properties(${name} PROPERTIES SUFFIX ".dylib")
    endif()
    if(WIN32)
        target_link_libraries(${name} PRIVATE ws2_32)
    endif()
endfunction()

# MC6850 ACIA (UART), relocatable via base=. Pure I/O -- no memory model.
add_i8085_plugin(mc6850)
# Memory-region plugin: ROM / write-protected EEPROM via the on_mem_write hook.
add_i8085_plugin(memory)
# Intel 8254 PIT: three free-running counters, self-describing via snapshot().
add_i8085_plugin(pit8254)
# Intel 8255 PPI: three parallel ports + control word, self-describing.
add_i8085_plugin(i8255)
# Generic live board viewer: renders every peer's snapshot() to a channel.
add_i8085_plugin(boardview)

# i8085-console: raw-mode stdin<->TCP bridge, auto-spawned as a per-UART window.
add_executable(i8085-console tools/i8085-console/i8085-console.cpp)
if(WIN32)
    target_link_libraries(i8085-console PRIVATE ws2_32)
endif()
if(UNIX)
    find_package(Threads REQUIRED)
    target_link_libraries(i8085-console PRIVATE Threads::Threads)
endif()
