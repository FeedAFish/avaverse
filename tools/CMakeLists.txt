cmake_minimum_required(VERSION 3.22)

add_executable(skinning skinning.cpp)
target_compile_features(skinning PUBLIC cxx_std_20)
target_link_libraries(skinning PUBLIC igl::glfw igl::png cxxopts::cxxopts)

add_executable(mesh2bbw mesh2bbw.cpp)
target_compile_features(mesh2bbw PUBLIC cxx_std_20)
target_link_libraries(mesh2bbw PUBLIC igl_copyleft::cgal igl_copyleft::tetgen cxxopts::cxxopts)
