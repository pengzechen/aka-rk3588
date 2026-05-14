cmake_minimum_required(VERSION 3.10)
project(aka_rk3588_tennis)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Log level ─────────────────────────────────────────────────────────────────
if(NOT DEFINED LOG_LEVEL)
  set(LOG_LEVEL "INFO")
endif()
if(LOG_LEVEL STREQUAL "DEBUG")
  add_definitions(-DCURRENT_LOG_LEVEL=0)
elseif(LOG_LEVEL STREQUAL "INFO")
  add_definitions(-DCURRENT_LOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "WARN")
  add_definitions(-DCURRENT_LOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "ERROR")
  add_definitions(-DCURRENT_LOG_LEVEL=3)
else()
  add_definitions(-DCURRENT_LOG_LEVEL=1)
endif()

# ── yolov8 tree: 3rdparty sets LIBRKNNRT, LIBRKNNRT_INCLUDES, LIBJPEG_INCLUDES ─
set(YOLOV8_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../yolov8)

if(NOT DEFINED TARGET_SOC)
  set(TARGET_SOC rk3588)
endif()

# ── NATIVE_BUILD: compile directly on the board (no cross-toolchain needed) ──
option(NATIVE_BUILD "Build natively on the board using system libraries" OFF)

# ── RKNN YOLOv8 sources (rknpu2 for rk3588) ──────────────────────────────────
set(RKNPU2_SRC
    ${YOLOV8_ROOT}/examples/yolov8/cpp/postprocess.cc
    ${YOLOV8_ROOT}/examples/yolov8/cpp/rknpu2/yolov8.cc
)

if(NATIVE_BUILD)
  # ── Board native build ─────────────────────────────────────────────────────
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUVC  REQUIRED libuvc)
  pkg_check_modules(LIBUSB  REQUIRED libusb-1.0)
  pkg_check_modules(LIBJPEG REQUIRED libturbojpeg)

  # librknnrt shipped in 3rdparty tree
  set(RKNNRT_DIR         ${YOLOV8_ROOT}/3rdparty/rknpu2)
  set(LIBRKNNRT          ${RKNNRT_DIR}/Linux/aarch64/librknnrt.so)
  set(LIBRKNNRT_INCLUDES ${RKNNRT_DIR}/include)

  # utils/CMakeLists.txt needs these vars before add_subdirectory
  set(STB_INCLUDES        ${YOLOV8_ROOT}/3rdparty/stb_image)
  set(LIBJPEG_INCLUDES    ${LIBJPEG_INCLUDE_DIRS})
  set(LIBJPEG             ${LIBJPEG_LIBRARIES})
  set(LIBRGA              "")   # no RGA .so needed → CPU fallback
  set(LIBRGA_INCLUDES     ${YOLOV8_ROOT}/3rdparty/librga/include)  # headers still needed for im2d.h
  add_definitions(-DDISABLE_RGA)

  # tennis.rknn is a single-class model (tennis ball only): output=[1,5,N]
  # Override the default COCO 80-class value so postprocess parses correctly.
  add_definitions(-DOBJ_CLASS_NUM=1)

  # libsndfile: needed by audio_utils.c header inclusion
  pkg_check_modules(LIBSNDFILE REQUIRED sndfile)
  set(LIBSNDFILE          ${LIBSNDFILE_LIBRARIES})
  set(LIBSNDFILE_INCLUDES ${LIBSNDFILE_INCLUDE_DIRS})

  add_subdirectory(${YOLOV8_ROOT}/utils utils.out)

  include_directories(
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${LIBRKNNRT_INCLUDES}
      ${YOLOV8_ROOT}/examples/yolov8/cpp
      ${YOLOV8_ROOT}/utils
      ${LIBUVC_INCLUDE_DIRS}
      ${LIBUSB_INCLUDE_DIRS}
      ${LIBJPEG_INCLUDE_DIRS}
  )

  add_executable(tennis
      tennis.cpp
      motor/motor.cpp motor/pwm_motor_driver.cpp motor/uart_motor_driver.cpp
      capture/uvc_capture.cpp
      detect/detect.cpp
      ${RKNPU2_SRC}
  )

  target_link_libraries(tennis
      imageutils fileutils imagedrawing
      ${LIBRKNNRT}
      ${LIBUVC_LIBRARIES}
      ${LIBUSB_LIBRARIES}
      ${LIBJPEG_LIBRARIES}
      -lpthread -ldl -lm
  )

else()
  # ── Cross-compile on PC ────────────────────────────────────────────────────
  add_subdirectory(${YOLOV8_ROOT}/3rdparty 3rdparty.out)
  add_subdirectory(${YOLOV8_ROOT}/utils    utils.out)

  set(LIBJPEG_DIR ${YOLOV8_ROOT}/3rdparty/jpeg_turbo)

  include_directories(
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${LIBRKNNRT_INCLUDES}
      ${YOLOV8_ROOT}/examples/yolov8/cpp
      ${YOLOV8_ROOT}/utils
      ${LIBJPEG_DIR}/include
      ${YOLOV8_ROOT}/3rdparty/libuvc/include
      /usr/include   # fallback for libuvc/libuvc.h
  )

  add_executable(tennis
      tennis.cpp
      motor/motor.cpp motor/pwm_motor_driver.cpp motor/uart_motor_driver.cpp
      capture/uvc_capture.cpp
      detect/detect.cpp
      ${RKNPU2_SRC}
  )

  # Generate empty stub .so so the linker can resolve -luvc etc.
  foreach(STUBLIB uvc usb-1.0 turbojpeg)
      set(STUB_SRC ${CMAKE_CURRENT_BINARY_DIR}/stub_${STUBLIB}.c)
      set(STUB_SO  ${CMAKE_CURRENT_BINARY_DIR}/lib${STUBLIB}.so)
      file(WRITE ${STUB_SRC} "/* stub */\n")
      add_custom_command(
          OUTPUT  ${STUB_SO}
          COMMAND ${CMAKE_C_COMPILER} -shared -fPIC -o ${STUB_SO} ${STUB_SRC}
          DEPENDS ${STUB_SRC}
          COMMENT "Generating stub lib${STUBLIB}.so"
      )
      list(APPEND STUB_SO_LIST ${STUB_SO})
  endforeach()
  add_custom_target(stub_libs ALL DEPENDS ${STUB_SO_LIST})
  add_dependencies(tennis stub_libs)

  target_link_options(tennis PRIVATE
      -Wl,--allow-shlib-undefined
      -Wl,--unresolved-symbols=ignore-all
      -L${CMAKE_CURRENT_BINARY_DIR}
  )

  target_link_libraries(tennis
      imageutils fileutils imagedrawing
      ${LIBRKNNRT}
      -Wl,--no-as-needed
      -luvc -lusb-1.0 -lturbojpeg
      -Wl,--as-needed
      -lpthread -ldl -lm
  )
endif()

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS tennis DESTINATION bin)
