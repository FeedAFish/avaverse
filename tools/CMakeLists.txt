cmake_minimum_required(VERSION 3.22)

if("${LIBIGL_WITH_GLFW}")
  add_executable(skinning skinning.cpp)
  target_link_libraries(skinning PUBLIC igl::glfw)

  add_executable(texture texture.cpp)
  target_link_libraries(texture PUBLIC igl::glfw igl::png)
endif()
