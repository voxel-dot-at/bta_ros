# - Try to find btatofapi headers and binaries
# Once done this will define
#
set(BTA_ETH OFF)
set(BTA_P100 OFF)

FIND_PATH(bta_INCLUDE_DIR NAMES bta.h
 	PATHS
  		/usr/include/libbta/
		/usr/local/include/libbta/
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

find_library(bta_p100_LIBRARY NAMES bta_p100
        PATHS
  		/usr/lib/
		/usr/local/lib/
		../windows/lib/${ARCH_DIR}/
	NO_DEFAULT_PATH
	DOC "Library binary"
)

if (BTA_ETH)
	set(bta_LIBRARIES ${bta_eth_LIBRARY})
elseif (BTA_P100)
	set(bta_LIBRARIES ${bta_eth_LIBRARY})
else ()
set(bta_LIBRARIES ${bta_p100_LIBRARY} ${bta_eth_LIBRARY})
endif()
#set(m100_INCLUDE_DIRS ${m100_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set bta_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(bta  DEFAULT_MSG
                                  bta_LIBRARIES bta_INCLUDE_DIR)

mark_as_advanced(bta_INCLUDE_DIR bta_LIBRARIES)


