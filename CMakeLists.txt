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

option(UAC_PSTV_ENABLE_LOGGING
  "Compile debug/file logging into the PSTV modules" OFF)

add_executable(uac_pstv_boot_kernel
  src/core_main.c
  src/stream.c
  src/uac1.c
)

target_compile_options(uac_pstv_boot_kernel PRIVATE
  -Wall
  -Wextra
  -Werror
  -O2
  -mtune=cortex-a9
)

target_link_libraries(uac_pstv_boot_kernel
  SceCpuForDriver_stub
  SceSysclibForDriver_stub
  SceSysmemForDriver_stub
  SceThreadmgrForDriver_stub
  SceUsbdForDriver_stub
)

if(UAC_PSTV_ENABLE_LOGGING)
  target_sources(uac_pstv_boot_kernel PRIVATE src/log.c)
  target_compile_definitions(uac_pstv_boot_kernel PRIVATE
    UAC_PSTV_ENABLE_LOGGING=1)
  target_link_libraries(uac_pstv_boot_kernel
    SceDebugForDriver_stub
    SceIofilemgrForDriver_stub
  )
endif()

set_target_properties(uac_pstv_boot_kernel PROPERTIES LINK_FLAGS "-nostdlib")

vita_create_self(uac_pstv_boot.skprx uac_pstv_boot_kernel
  CONFIG ${CMAKE_SOURCE_DIR}/uac_pstv_boot.yml
  UNSAFE
)

vita_create_stubs(uac_pstv_boot_stubs uac_pstv_boot_kernel
  ${CMAKE_SOURCE_DIR}/uac_pstv_boot.yml KERNEL)

link_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/uac_pstv_boot_stubs
)

add_executable(uac_pstv_audio_kernel
  src/main.c
  src/audio.c
  src/mixer.c
)

add_dependencies(uac_pstv_audio_kernel uac_pstv_boot_stubs)

target_compile_options(uac_pstv_audio_kernel PRIVATE
  -Wall
  -Wextra
  -Werror
  -O2
	-mtune=cortex-a9
)

target_link_libraries(uac_pstv_audio_kernel
  SceCpuForDriver_stub
  SceKernelSuspendForDriver_stub
  SceSysclibForDriver_stub
  SceSysmemForDriver_stub
  SceThreadmgrForDriver_stub
  SceUsbServForDriver_stub
  taihenForKernel_stub
  uacPstvBootForDriver_stub
)

if(UAC_PSTV_ENABLE_LOGGING)
  target_sources(uac_pstv_audio_kernel PRIVATE src/log.c)
  target_compile_definitions(uac_pstv_audio_kernel PRIVATE
    UAC_PSTV_ENABLE_LOGGING=1)
  target_link_libraries(uac_pstv_audio_kernel
    SceDebugForDriver_stub
    SceIofilemgrForDriver_stub
  )
endif()

set_target_properties(uac_pstv_audio_kernel PROPERTIES LINK_FLAGS "-nostdlib")

vita_create_self(uac_pstv_audio.skprx uac_pstv_audio_kernel
  CONFIG ${CMAKE_SOURCE_DIR}/uac_pstv_audio.yml
  UNSAFE
)
