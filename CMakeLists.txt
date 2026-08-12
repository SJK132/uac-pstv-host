cmake_minimum_required(VERSION 3.10)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake"
        CACHE PATH "VitaSDK toolchain file")
  else()
    message(FATAL_ERROR "Set VITASDK before configuring this project")
  endif()
endif()

project(uac_pstv C)
include("${VITASDK}/share/vita.cmake" REQUIRED)

option(UAC_PSTV_ENABLE_LOGGING "Enable file and debug logging" OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_executable(uac_pstv_kernel
  src/main.c
  src/uac1.c
  src/stream.c
  src/audio_tap.c
  src/resolver.c
)

target_include_directories(uac_pstv_kernel PRIVATE src)
target_compile_options(uac_pstv_kernel PRIVATE
  -Wall -Wextra -Werror -O2 -mtune=cortex-a9
)

target_link_libraries(uac_pstv_kernel
  SceCpuForDriver_stub
  SceKernelSuspendForDriver_stub
  SceSblAIMgrForDriver_stub
  SceSysclibForDriver_stub
  SceThreadmgrForDriver_stub
  SceUsbServForDriver_stub
  SceUsbdForDriver_stub
  taihenForKernel_stub
)

if(UAC_PSTV_ENABLE_LOGGING)
  target_sources(uac_pstv_kernel PRIVATE src/log.c)
  target_compile_definitions(uac_pstv_kernel PRIVATE UAC_PSTV_ENABLE_LOGGING=1)
  target_link_libraries(uac_pstv_kernel
    SceDebugForDriver_stub
    SceIofilemgrForDriver_stub
  )
endif()

set_target_properties(uac_pstv_kernel PROPERTIES LINK_FLAGS "-nostdlib")
set(VITA_ELF_CREATE_FLAGS "" CACHE STRING "vita-elf-create flags" FORCE)
set(VITA_MAKE_FSELF_FLAGS "" CACHE STRING "vita-make-fself flags" FORCE)

vita_create_self(uac_pstv.skprx uac_pstv_kernel
  CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/uac_pstv.yml
  UNSAFE
)
