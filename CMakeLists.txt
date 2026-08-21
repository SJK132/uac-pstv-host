cmake_minimum_required(VERSION 3.10)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake"
        CACHE PATH "VitaSDK toolchain file")
  else()
    message(FATAL_ERROR "Set VITASDK before configuring this project")
  endif()
endif()

project(uac_pstv VERSION 1.3.0 LANGUAGES C)
include("${VITASDK}/share/vita.cmake" REQUIRED)

option(UAC_PSTV_ENABLE_LOGGING "Enable file and debug logging" OFF)

# Build identity, banner-logged at module start so a log identifies the exact
# binary that produced it.  git describe carries the tag, the commit, and a
# -dirty suffix for a working tree that does not match any commit -- which is
# the state most test builds are in.  Resolved at configure time; build.sh
# always configures from scratch, so it cannot go stale there.
set(UAC_PSTV_BUILD_REV "unknown")
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" describe --always --dirty --abbrev=8
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE UAC_PSTV_GIT_REV
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(UAC_PSTV_GIT_REV)
    set(UAC_PSTV_BUILD_REV "${UAC_PSTV_GIT_REV}")
  endif()
endif()
message(STATUS "uac-pstv ${PROJECT_VERSION} (${UAC_PSTV_BUILD_REV})")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_executable(uac_pstv_kernel
  src/main.c
  src/uac1.c
  src/session.c
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
  # Scoped to the logging build on purpose: the release binary cannot be
  # perturbed by a commit or a dirty tree, so its hash stays reproducible.
  target_compile_definitions(uac_pstv_kernel PRIVATE
    UAC_PSTV_ENABLE_LOGGING=1
    UAC_PSTV_VERSION="${PROJECT_VERSION}"
    UAC_PSTV_BUILD_REV="${UAC_PSTV_BUILD_REV}")
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
