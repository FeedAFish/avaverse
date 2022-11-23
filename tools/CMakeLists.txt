cmake_minimum_required(VERSION 3.22)

add_executable(skinning-bin skinning.cpp)
target_compile_features(skinning-bin PUBLIC cxx_std_20)
target_link_libraries(skinning-bin PUBLIC igl::glfw igl::png cxxopts::cxxopts)
target_link_libraries(skinning-bin PUBLIC skinning)
set_target_properties(skinning-bin PROPERTIES RUNTIME_OUTPUT_NAME skinning)

add_executable(mesh2bbw-bin mesh2bbw.cpp)
target_compile_features(mesh2bbw-bin PUBLIC cxx_std_20)
target_link_libraries(mesh2bbw-bin PUBLIC igl_copyleft::cgal igl_copyleft::tetgen cxxopts::cxxopts)
set_target_properties(mesh2bbw-bin PROPERTIES RUNTIME_OUTPUT_NAME mesh2bbw)
