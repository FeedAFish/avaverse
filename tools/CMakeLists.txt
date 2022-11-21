cmake_minimum_required(VERSION 3.22)

if("${LIBIGL_WITH_GLFW}")
  add_executable(skinning skinning.cpp)
  target_link_libraries(skinning PUBLIC igl::glfw igl::png)
endif()
