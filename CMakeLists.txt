cmake_minimum_required(VERSION 3.8)
project(jif)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

cmake_policy(SET CMP0072 NEW)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.3 REQUIRED)

file(GLOB jif_src CONFIGURE_DEPENDS "src/jif/*.cpp" "src/jif/layout/*.cpp" "src/imgui/*.cpp" "src/imgui/backends/*.cpp" "src/imgui/misc/*.cpp" "src/kubazip/*.c")

add_executable(jif-core ${jif_src})
target_include_directories(jif-core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jif-core PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(jif-core
  OpenGL::GL
  GLEW::GLEW
  glfw
)

install(TARGETS jif-core
  DESTINATION lib/${PROJECT_NAME})

install(FILES $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/res>
  DESTINATION lib/${PROJECT_NAME})

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
