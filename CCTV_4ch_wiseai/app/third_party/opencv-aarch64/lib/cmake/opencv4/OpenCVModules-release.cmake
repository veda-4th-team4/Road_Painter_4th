#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "zlib" for configuration "Release"
set_property(TARGET zlib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(zlib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/libzlib.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS zlib )
list(APPEND _IMPORT_CHECK_FILES_FOR_zlib "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/libzlib.a" )

# Import target "libopenjp2" for configuration "Release"
set_property(TARGET libopenjp2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(libopenjp2 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/liblibopenjp2.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS libopenjp2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_libopenjp2 "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/liblibopenjp2.a" )

# Import target "quirc" for configuration "Release"
set_property(TARGET quirc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(quirc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/libquirc.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS quirc )
list(APPEND _IMPORT_CHECK_FILES_FOR_quirc "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/libquirc.a" )

# Import target "tegra_hal" for configuration "Release"
set_property(TARGET tegra_hal APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(tegra_hal PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/libtegra_hal.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS tegra_hal )
list(APPEND _IMPORT_CHECK_FILES_FOR_tegra_hal "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/libtegra_hal.a" )

# Import target "ade" for configuration "Release"
set_property(TARGET ade APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ade PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/libade.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ade )
list(APPEND _IMPORT_CHECK_FILES_FOR_ade "${_IMPORT_PREFIX}/lib/opencv4/3rdparty/libade.a" )

# Import target "opencv_core" for configuration "Release"
set_property(TARGET opencv_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopencv_core.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_core "${_IMPORT_PREFIX}/lib/libopencv_core.a" )

# Import target "opencv_flann" for configuration "Release"
set_property(TARGET opencv_flann APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_flann PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopencv_flann.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_flann )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_flann "${_IMPORT_PREFIX}/lib/libopencv_flann.a" )

# Import target "opencv_imgproc" for configuration "Release"
set_property(TARGET opencv_imgproc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_imgproc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopencv_imgproc.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_imgproc )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_imgproc "${_IMPORT_PREFIX}/lib/libopencv_imgproc.a" )

# Import target "opencv_features2d" for configuration "Release"
set_property(TARGET opencv_features2d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_features2d PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopencv_features2d.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_features2d )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_features2d "${_IMPORT_PREFIX}/lib/libopencv_features2d.a" )

# Import target "opencv_calib3d" for configuration "Release"
set_property(TARGET opencv_calib3d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_calib3d PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopencv_calib3d.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_calib3d )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_calib3d "${_IMPORT_PREFIX}/lib/libopencv_calib3d.a" )

# Import target "opencv_aruco" for configuration "Release"
set_property(TARGET opencv_aruco APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_aruco PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopencv_aruco.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_aruco )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_aruco "${_IMPORT_PREFIX}/lib/libopencv_aruco.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
