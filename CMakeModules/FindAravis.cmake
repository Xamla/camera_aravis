INCLUDE(FindPackageHandleStandardArgs)

FIND_PATH(Aravis_INCLUDE_DIRS arv.h
  "$ENV{Aravis_INCLUDE_DIRS}"
  /usr/local/include/aravis-0.4
)

FIND_LIBRARY(Aravis_LIBRARIES aravis-0.4
  "$ENV{Aravis_LIBRARIES}"
  /usr/local/lib
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(ARAVIS DEFAULT_MSG
  Aravis_INCLUDE_DIRS
  Aravis_LIBRARIES)

