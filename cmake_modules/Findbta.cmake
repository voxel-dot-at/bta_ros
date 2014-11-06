# - Try to find btatofapi headers and binaries
# Once done this will define
#
FIND_PATH(bta_INCLUDE_DIR NAMES bta.h
 	PATHS
  		/usr/include/libbta/
		/usr/local/include/libbta/
		/usr/local/include/
		../include/
		../inc/
		./
 	NO_DEFAULT_PATH
	DOC "Include directory of bta"
)

	if (CMAKE_CL_64)
      set(ARCH_DIR "x64")
  	else()
      set(ARCH_DIR "x86")
  	endif()

find_library(bta_eth_LIBRARY NAMES bta_eth
        PATHS
  		/usr/lib/
		/usr/local/lib/
		../windows/lib/${ARCH_DIR}/
	NO_DEFAULT_PATH
	DOC "Library binary"
)

#Not needed by now as we have just 1 include and lib
set(bta_LIBRARIES ${bta_eth_LIBRARY} )
#set(m100_INCLUDE_DIRS ${m100_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set bta_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(bta  DEFAULT_MSG
                                  bta_LIBRARIES bta_INCLUDE_DIR)

mark_as_advanced(bta_INCLUDE_DIR bta_LIBRARIES)


