#include <igl/directed_edge_parents.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/png/readPNG.h>
#include <igl/readPLY.h>
#include <igl/readTGF.h>

#include <Eigen/Geometry>
#include <cxxopts.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
  cxxopts::Options options("skinning", "Skinning on deformed mesh");
  auto options_adder = options.add_options();
  options_adder("help", "Print help");
  options_adder("m,mesh", "Mesh path (.ply)", cxxopts::value<std::string>());
  options_adder("t,texture", "Texture path (.png)",
                cxxopts::value<std::string>());
  options_adder("s,skeleton", "Skeleton path (.tgf)",
                cxxopts::value<std::string>());
  auto args = options.parse(argc, argv);

  if (args.count("help")) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  Eigen::MatrixXd V, N, UV;
  Eigen::MatrixXi F, E;
  igl::readPLY(args["mesh"].as<std::string>(), V, F, E, N, UV);

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A;
  igl::png::readPNG(args["texture"].as<std::string>(), R, G, B, A);

  Eigen::MatrixXd C;
  Eigen::MatrixXi BE;
  igl::readTGF(args["skeleton"].as<std::string>(), C, BE);

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

  return 0;
}
