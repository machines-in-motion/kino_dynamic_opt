#  Find GUROBI and set
#
#    GUROBI_FOUND        - System has Gurobi
#    GUROBI_INCLUDE_DIRS - The Gurobi include directories
#    GUROBI_LIBRARIES    - The libraries needed to use Gurobi

find_path( GUROBI_INCLUDE_DIR 
  NAMES gurobi_c++.h
  PATHS "/Library/gurobi811/mac64/include"
        "/Library/gurobi801/mac64/include"
        "$ENV{GUROBI_HOME}/include"
)

find_library( GUROBI_LIBRARY 
  NAMES gurobi81 gurobi80
  PATHS "/Library/gurobi811/mac64/lib"
        "/Library/gurobi801/mac64/lib"
        "$ENV{GUROBI_HOME}/lib"
)

find_library( GUROBI_CXX_LIBRARY 
  NAMES gurobi_c++
  PATHS "/Library/gurobi811/mac64/lib"
        "/Library/gurobi801/mac64/lib"
        "$ENV{GUROBI_HOME}/lib"
)

set( GUROBI_INCLUDE_DIRS  ${GUROBI_INCLUDE_DIR} )
set( GUROBI_LIBRARIES     ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG
  GUROBI_LIBRARY GUROBI_CXX_LIBRARY GUROBI_INCLUDE_DIR)

mark_as_advanced(GUROBI_INCLUDE_DIR GUROBI_LIBRARY GUROBI_CXX_LIBRARY)

add_library(GUROBI::GUROBI SHARED IMPORTED)
set_target_properties(GUROBI::GUROBI PROPERTIES
    IMPORTED_LOCATION ${GUROBI_LIBRARIES}
    INTERFACE_INCLUDE_DIRECTORIES ${GUROBI_INCLUDE_DIRS})
