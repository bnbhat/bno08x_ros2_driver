# sh2/CMakeLists.txt

# Create a library from the sh2 sources
add_library(sh2_library
  sh2.c
  sh2_SensorValue.c
  sh2_util.c
  shtp.c
)

# Set include directories for this library
target_include_directories(sh2_library PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR} # sh2/
)

# If any specific compile features or options are needed for the sh2 library
target_compile_features(sh2_library PUBLIC c_std_99)
