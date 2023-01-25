cmake_minimum_required(VERSION 3.8)
project(allegro_lib)

find_package(ament_cmake REQUIRED)
include_directories(include)

# find_library(NAMES libpcanbasic PATHS home/ritz/WinterQ/Project/AllegroHandManipulation/Allegro_interface/src/peak-linux-driver-8.15.2/libpcanbasic/pcanbasic/lib/libpcanbasic.so.4.6.2)
add_library(allegro_lib src/canAPI.cpp src/RockScissorsPaper.cpp)
target_link_libraries(allegro_lib pcanbasic)
# target_include_directories(allegro_lib
#                            PUBLIC
#                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
#                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
# )
ament_export_targets(allegro_lib HAS_LIBRARY_TARGET)
# The add_library function just added turtlelib as a "target"
# A "target" is a name that CMake uses to refer to some type of output
# In this case it is a library but it could also be an executable or some other items

# Public include directories can be used by other targets that link against turtlelib
# By adding include/ to the include path, this means that files in e.g., include/turtlelib
# can be included with #include"turtlelib/file.hpp"
# target_include_directories(allegro_lib PUBLIC include/)

# # enable C++ 17
# target_compile_features(allegro_lib PUBLIC cxx_std_17) 

# # warnings are your friend!
# target_compile_options(allegro_lib PUBLIC -Wall -Wextra -Wpedantic)

# ament_export_include_directories(include)

# ament_export_libraries(allegro_lib)
# # this line to exports the library
# ament_export_targets(allegro_lib HAS_LIBRARY_TARGET)

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/allegro_lib
  DESTINATION include
)

install(
  TARGETS allegro_lib
  EXPORT allegro_lib
  LIBRARY DESTINATION lib #${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION lib #${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION bin #${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION include #${CMAKE_INSTALL_INCLUDEDIR}
)
# create the executable target  and link it with the rigid2d library
# It is also possible specify multiple cpp files and they will be linked
# into a single executable (as long as exactly one of these files includes a main() function).
# However, by creating a library (as we are doing here) the library files
# can be compiled once and used

# install(FILES canAPI.h RockScissorsPaper.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# install(EXPORT allegro_libTargets
#         FILE allegro_libTargets.cmake
#         NAMESPACE allegro_lib::
#         DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/allegro_lib
# )
# add_library(allegro_lib::allegro_lib STATIC IMPORTED)

# set_target_properties(allegro_lib::allegro_lib PROPERTIES
#   INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
