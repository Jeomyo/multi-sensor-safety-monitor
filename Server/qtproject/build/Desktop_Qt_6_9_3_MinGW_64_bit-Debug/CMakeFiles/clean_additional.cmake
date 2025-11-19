# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles\\appqtproject_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\appqtproject_autogen.dir\\ParseCache.txt"
  "appqtproject_autogen"
  )
endif()
