if(TARGET igl::core)
  return()
endif()

set(LIBIGL_GIT_TAG 87a550af22fc4af210af40f1d61d81594bbaf546)
include(FetchContent)
FetchContent_Declare(
  libigl
  GIT_REPOSITORY https://github.com/libigl/libigl.git
  GIT_TAG "${LIBIGL_GIT_TAG}"
  PATCH_COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_ROOT_DIR}/cmake/external/libigl.patch" <SOURCE_DIR> && git reset --hard "${LIBIGL_GIT_TAG}" && git apply <SOURCE_DIR>/libigl.patch
)
FetchContent_MakeAvailable(libigl)
