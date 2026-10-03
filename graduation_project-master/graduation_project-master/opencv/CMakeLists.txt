cmake_minimum_required(VERSION 3.16)
project(opencv_lane CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------- OpenCV
# Point this at your OpenCV install if find_package can't locate it:
#   cmake -B build -DOpenCV_DIR="C:/opencv/opencv/build/x64/vc16/lib"
set(OpenCV_DIR "" CACHE PATH "Directory containing OpenCVConfig.cmake")

find_package(OpenCV REQUIRED COMPONENTS
  core
  imgproc
  imgcodecs
  videoio
  calib3d
  highgui
  dnn
)
message(STATUS "OpenCV ${OpenCV_VERSION} -> ${OpenCV_INCLUDE_DIRS}")

# ---------------------------------------------------------------- sources
# Only the files that the original Xcode target actually compiled.
#   excluded: camera.cpp     (fragment, no main, will not compile)
#   excluded: curveTemp.cpp  (100% commented out)
#   excluded: LaneDetection.cpp (legacy, unused by the pipeline)
add_executable(opencv
  main.cpp
  AdvancedLaneDetection.cpp
  Calibration.cpp
  ObjectDetection.cpp
  Utility.cpp
)

target_include_directories(opencv PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv PRIVATE ${OpenCV_LIBS})

if(MSVC)
  target_compile_options(opencv PRIVATE /utf-8 /W3)
  # OpenCV prebuilt binaries are built with a specific runtime; match it.
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
else()
  target_compile_options(opencv PRIVATE -Wall)
endif()

# ---------------------------------------------------------------- runtime dlls
# The prebuilt OpenCV is an import lib against opencv_world4110.dll, so the
# DLLs have to sit next to the exe (or on PATH) at run time.
if(WIN32)
  # include/ lives at <root>/build/include and the import DLLs at
  # <root>/build/x64/vc16/bin, so the bin dir is one level up from include.
  get_filename_component(OpenCV_BIN_DIR "${OpenCV_INCLUDE_DIRS}/../x64/vc16/bin" ABSOLUTE)
  if(EXISTS "${OpenCV_BIN_DIR}")
    message(STATUS "Staging OpenCV runtime DLLs from ${OpenCV_BIN_DIR}")
    file(GLOB OpenCV_RUNTIME_DLLS "${OpenCV_BIN_DIR}/*.dll")
    add_custom_command(TARGET opencv POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${OpenCV_RUNTIME_DLLS} "$<TARGET_FILE_DIR:opencv>"
      VERBATIM)
  else()
    message(WARNING "OpenCV bin dir not found at ${OpenCV_BIN_DIR}; "
                    "add it to PATH manually or the exe will not start")
  endif()
endif()

# ---------------------------------------------------------------- code signing
# Windows Smart App Control refuses to launch unsigned binaries from a local
# build directory, so every rebuild looks "untrusted" and the freshly linked
# exe fails to start until it is signed. Sign it after linking when a dev
# certificate is available; skip silently otherwise so the build still works on
# machines that do not need it.
#
#   New-SelfSignedCertificate -Type CodeSigningCert -Subject "CN=LocalDev" `
#     -CertStoreLocation Cert:\CurrentUser\My -KeyUsage DigitalSignature
#   certutil -user -addstore Root         <cert.cer>
#   certutil -user -addstore TrustedPublisher <cert.cer>
#
set(DEV_SIGN_PFX "$ENV{LOCALAPPDATA}/opencv-build/localdev.pfx" CACHE FILEPATH
    "PFX used to sign the built exe so Smart App Control will launch it")
set(DEV_SIGN_PASSWORD "opencode" CACHE STRING "Password for DEV_SIGN_PFX")

if(WIN32 AND EXISTS "${DEV_SIGN_PFX}")
  find_program(SIGNTOOL signtool HINTS
    "C:/Program Files (x86)/Windows Kits/10/bin/10.0.26100.0/x64"
    "C:/Program Files (x86)/Windows Kits/10/bin/10.0.22621.0/x64")
  if(SIGNTOOL)
    message(STATUS "Signing with ${DEV_SIGN_PFX}")
    add_custom_command(TARGET opencv POST_BUILD
      COMMAND "${SIGNTOOL}" sign /f "${DEV_SIGN_PFX}" /p "${DEV_SIGN_PASSWORD}"
              /fd SHA256 "$<TARGET_FILE:opencv>"
      VERBATIM)
  else()
    message(STATUS "signtool not found; exe will be left unsigned "
                   "(Smart App Control may block it)")
  endif()
endif()

# ---------------------------------------------------------------- convenience
# `cmake --build build --target run_sobel` renders the default clip.
add_custom_target(run_sobel
  COMMAND $<TARGET_FILE:opencv>
          --video "${CMAKE_CURRENT_SOURCE_DIR}/../Advanced-Lane-Lines-master/project_video.mp4"
          --out   "${CMAKE_CURRENT_SOURCE_DIR}/../output_cpp"
  DEPENDS opencv
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
  COMMENT "Running lane detection on project_video.mp4"
  USES_TERMINAL
)
