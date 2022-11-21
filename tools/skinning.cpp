#include <igl/directed_edge_parents.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/png/readPNG.h>
#include <igl/readPLY.h>
#include <igl/readTGF.h>

#include <Eigen/Geometry>

int main(int argc, char *argv[]) {
  Eigen::MatrixXd V, N, UV;
  Eigen::MatrixXi F, E;
  igl::readPLY(argv[1], V, F, E, N, UV);

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A;
  igl::png::readPNG(argv[2], R, G, B, A);

  Eigen::MatrixXd C;
  Eigen::MatrixXi BE;
  igl::readTGF(argv[3], C, BE);

  Eigen::VectorXi P;
  igl::directed_edge_parents(BE, P);

  const Eigen::RowVector3d edge_color(70. / 255., 252. / 255., 167. / 255.);

  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);
  viewer.data().set_normals(N);
  viewer.data().set_uv(UV);
  viewer.data().show_texture = true;
  viewer.data().set_texture(R, G, B, A);
  viewer.data().set_edges(C, BE, edge_color);
  viewer.launch();

  return EXIT_SUCCESS;
}
