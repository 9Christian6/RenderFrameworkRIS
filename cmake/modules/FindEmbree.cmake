find_path(Embree_INCLUDE_DIR embree3/rtcore.h
  /usr/include
  /usr/local/include
  /opt/local/include)

find_library(Embree_LIBRARY NAMES embree3 PATHS 
  /usr/lib 
  /usr/local/lib 
  /opt/local/lib)

if (Embree_INCLUDE_DIR AND Embree_LIBRARY)
    set(Embree_FOUND ON)
endif ()
