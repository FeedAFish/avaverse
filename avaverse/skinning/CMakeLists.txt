# Skinning
project(skinning)

add_library(${PROJECT_NAME} src/skinning.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_link_libraries(${PROJECT_NAME} PUBLIC common)
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw igl::png)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/include
)
