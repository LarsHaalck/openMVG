#---------------------
#Third party libraries
#---------------------

# libs should be static
set(BUILD_SHARED_LIBS OFF)

# use PIC code for link into shared lib
if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif(UNIX)

if (MSVC)
  # Disable warning about size_t conversions for all third_party libraries.
  add_compile_options(/wd4267)
endif()

# Unit test library
if (OpenMVG_BUILD_TESTS)
  add_subdirectory(CppUnitLite)
endif()

# Basic filesystem utils
set(STLPLUS_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/openMVG/third_party/stlplus3)
add_subdirectory(stlplus3)

# Add graph library
if(DEFINED OpenMVG_USE_INTERNAL_LEMON)
  add_subdirectory(lemon)
  set_property(TARGET lemon PROPERTY FOLDER OpenMVG/3rdParty/lemon)
  set_property(TARGET check PROPERTY FOLDER OpenMVG/3rdParty/lemon)
endif()

# Add svg library
add_subdirectory(vectorGraphics)

# Exif data parsing
set(EASYEXIF_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/openMVG/third_party/easyexif)
add_subdirectory(easyexif)

# Fast corner detector
set(FAST_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/openMVG/third_party/fast)
add_subdirectory(fast)

list(APPEND directories cmdLine histogram htmlDoc progress vectorGraphics)
foreach(inDirectory ${directories})
install(
  DIRECTORY ./${inDirectory}
  DESTINATION include/openMVG/third_party/
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
endforeach(inDirectory)
