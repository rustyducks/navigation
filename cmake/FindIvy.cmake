INCLUDE(FindPackageHandleStandardArgs)

SET(IVY_IncludeSearchPaths
  /usr/include/
  /usr/local/include/
  /opt/local/include/
)

SET(GLIBIVY_IncludeSearchPaths
  /usr/include/
  /usr/local/include/
  /opt/local/include/
)

SET(IVY_LibrarySearchPaths
  /usr/lib/
  /usr/local/lib/
  /opt/local/lib/
)

FIND_PATH(Ivy_INCLUDE_DIRS Ivy/ivy.h
  PATHS ${Ivy_IncludeSearchPaths}
)
FIND_LIBRARY(Ivy_LIBRARY_OPTIMIZED
  NAMES ivy
  PATHS ${Ivy_LibrarySearchPaths}
)


# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
FIND_PACKAGE_HANDLE_STANDARD_ARGS(IVY "Could NOT find libivy library (IVY)"
  Ivy_LIBRARY_OPTIMIZED
  Ivy_INCLUDE_DIRS
)

# Collect optimized and debug libraries
#HANDLE_LIBRARY_TYPES(IVY)
#HANDLE_LIBRARY_TYPES(GLIBIVY)

MARK_AS_ADVANCED(
  Ivy_INCLUDE_DIRS
  Ivy_LIBRARY_OPTIMIZED
)