cmake_minimum_required(VERSION 3.8)
project(allegro_lib)

find_package(ament_cmake REQUIRED)
include_directories(include)

add_library(allegro_lib src/canAPI.cpp src/RockScissorsPaper.cpp)
target_link_libraries(allegro_lib pcanbasic)

ament_export_targets(allegro_lib HAS_LIBRARY_TARGET)


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
