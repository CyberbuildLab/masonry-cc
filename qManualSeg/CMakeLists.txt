cmake_minimum_required( VERSION 3.0 )

# CloudCompare Masonry Manual Segmentation plugin
option( PLUGIN_STANDARD_MASONRY_QMANUAL_SEG "Check to install qManualSeg plugin" OFF )

if ( PLUGIN_STANDARD_MASONRY_QMANUAL_SEG )

    project( QMANUAL_SEG_PLUGIN )
    
    AddPlugin( NAME ${PROJECT_NAME} )


	target_sources( ${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_LIST_DIR}/qManualSeg.h
			${CMAKE_CURRENT_LIST_DIR}/qManualSeg.cpp
	)

	target_include_directories( ${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}
	)
 
	# Find Packages
	# Find PCL
	#set( PCL_DIR "C:/Program Files/PCL 1.9.1") #DGM: can't do that as it will obviously conflict with most of the users settings ;)
	find_package( PCL 1.9.1 REQUIRED )

	# Find OpenCV
	#set( OpenCV_DIR "C:/opencv/build" ) #DGM: can't do that as it will obviously conflict with most of the users settings ;)
	find_package( OpenCV REQUIRED )

	if (WIN32)
		# We need to copy the OpenCV "World" DLL file next to CloudCompare.exe
		set( OpenCV_WORLD_DLL "" CACHE FILEPATH "OpenCV 'World' DLL file path" )
		if( CMAKE_CONFIGURATION_TYPES )
			set( OpenCV_WORLD_DEBUG_DLL "" CACHE FILEPATH "OpenCV Debug 'World' DLL file path" )
		endif()
	endif()

	if( PCL_FOUND AND OpenCV_FOUND )
		# [C/C++]>[General]>[Additional Include Directories](
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

		if (WIN32)
			if (OpenCV_WORLD_DLL)
				copy_files("${OpenCV_WORLD_DLL}" ${CLOUDCOMPARE_DEST_FOLDER} 0)
			endif()
			if (CMAKE_CONFIGURATION_TYPES AND OpenCV_WORLD_DEBUG_DLL)
				copy_files("${OpenCV_WORLD_DEBUG_DLL}" ${CLOUDCOMPARE_DEST_FOLDER} 2)
			endif()
		endif()

	endif()

endif()
