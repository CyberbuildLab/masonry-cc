cmake_minimum_required( VERSION 3.0 )

# CloudCompare Masonry Manual Segmentation plugin
option( INSTALL_QMANUAL_SEG_PLUGIN "Check to install qManualSeg plugin" OFF )

if ( INSTALL_QMANUAL_SEG_PLUGIN )

    project( QMANUAL_SEG_PLUGIN )
    
	#include( ../../CMakePluginTpl.cmake )
    AddPlugin( NAME ${PROJECT_NAME} )


target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qManualSeg.h
		${CMAKE_CURRENT_LIST_DIR}/qManualSeg.cpp
		${CMAKE_CURRENT_LIST_DIR}/profileImportDlg.h
		${CMAKE_CURRENT_LIST_DIR}/profileImportDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/profileImportDlg.ui
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
 
	# Find Packages
	# Find PCL
	set( PCL_DIR "C:/Program Files/PCL 1.9.1")
	find_package( PCL 1.9.1 REQUIRED )

	# Find OpenCV
	set( OpenCV_DIR "C:/opencv/build" )
	find_package( OpenCV REQUIRED )

	if( PCL_FOUND AND OpenCV_FOUND )
	  # [C/C++]>[General]>[Additional Include Directories]
	  include_directories( ${PCL_INCLUDE_DIRS} )
	  include_directories( ${OpenCV_INCLUDE_DIRS} )

	  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
	  add_definitions( ${PCL_DEFINITIONS} )

	  # For Use Not PreCompiled Features 
	  #add_definitions( -DPCL_NO_PRECOMPILE )

	  # [Linker]>[General]>[Additional Library Directories]
	  link_directories( ${PCL_LIBRARY_DIRS} )
	  link_directories( ${OpenCV_LIB_DIR} )

	  # [Linker]>[Input]>[Additional Dependencies]
	  target_link_libraries( QMANUAL_SEG_PLUGIN ${PCL_LIBRARIES} )
	  target_link_libraries( QMANUAL_SEG_PLUGIN ${OpenCV_LIBS} )
	endif()

endif()