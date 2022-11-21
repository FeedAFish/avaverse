if(TARGET igl::core)
  return()
endif()

include(FetchContent)
FetchContent_Declare(
  libigl
  GIT_REPOSITORY https://github.com/libigl/libigl.git
  GIT_TAG v2.4.0
  PATCH_COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_ROOT_DIR}/cmake/external/libigl.patch" <SOURCE_DIR> && git apply <SOURCE_DIR>/libigl.patch
)
FetchContent_MakeAvailable(libigl)
