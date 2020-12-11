#  Find IPOPT within location(${IPOPT_DIR}) and set
#
#    IPOPT_FOUND        - If false, don't try to use IPOPT#    
#    IPOPT_INCLUDE_DIRS - Directories to include to use IPOPT
#    IPOPT_LIBRARIES    - Default library to link against to use IPOPT

#=============================================================================
# This find script is copied from https://github.com/robotology/idyntree.
# Original copyright notice can be seen below:
#=============================================================================
# Copyright (C) 2008-2010 RobotCub Consortium
# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
#   Authors: Ugo Pattacini <ugo.pattacini@iit.it>
#   Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of YCM, substitute the full
#  License text for the above reference.)

if(UNIX)

  if(DEFINED ENV{IPOPT_DIR})
    set(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")

    set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/include/coin)
    find_library(IPOPT_LIBRARIES ipopt ${IPOPT_DIR}/lib ${IPOPT_DIR}/lib/coin NO_DEFAULT_PATH)

    if(IPOPT_LIBRARIES)
      find_file(IPOPT_DEP_FILE ipopt_addlibs_cpp.txt ${IPOPT_DIR}/share/doc/coin/Ipopt
                                                     ${IPOPT_DIR}/share/coin/doc/Ipopt
                                                     NO_DEFAULT_PATH)
      mark_as_advanced(IPOPT_DEP_FILE)

      if(IPOPT_DEP_FILE)
        # parse the file and acquire the dependencies
        file(READ ${IPOPT_DEP_FILE} IPOPT_DEP)
        string(REGEX REPLACE "-[^l][^ ]* " "" IPOPT_DEP ${IPOPT_DEP})
        string(REPLACE "-l"                "" IPOPT_DEP ${IPOPT_DEP})
        string(REPLACE "\n"                "" IPOPT_DEP ${IPOPT_DEP})
        string(REPLACE "ipopt"             "" IPOPT_DEP ${IPOPT_DEP})
        separate_arguments(IPOPT_DEP)

        # use the find_library command in order to prepare rpath correctly
        foreach(LIB ${IPOPT_DEP})
          find_library(IPOPT_SEARCH_FOR_${LIB} ${LIB} ${IPOPT_DIR}/lib
                                                      ${IPOPT_DIR}/lib/coin
                                                      ${IPOPT_DIR}/lib/coin/ThirdParty
                                                      NO_DEFAULT_PATH)
          if(IPOPT_SEARCH_FOR_${LIB})
            # handle non-system libraries (e.g. coinblas)
            set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${IPOPT_SEARCH_FOR_${LIB}})
          else()
            # handle system libraries (e.g. gfortran)
            if (NOT APPLE)
              set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${LIB})
            endif()
          endif()
          mark_as_advanced(IPOPT_SEARCH_FOR_${LIB})
        endforeach()
      endif()
    endif()
  endif()
endif()

mark_as_advanced(IPOPT_INCLUDE_DIRS IPOPT_LIBRARIES IPOPT_DEFINITIONS IPOPT_LINK_FLAGS)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(IPOPT DEFAULT_MSG IPOPT_LIBRARIES)

add_library(IPOPT::IPOPT SHARED IMPORTED)
set_target_properties(IPOPT::IPOPT PROPERTIES
    IMPORTED_LOCATION ${IPOPT_LIBRARIES}
    INTERFACE_INCLUDE_DIRECTORIES ${IPOPT_INCLUDE_DIRS})
