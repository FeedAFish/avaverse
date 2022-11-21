#include <igl/directed_edge_parents.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/png/readPNG.h>
#include <igl/png/writePNG.h>
#include <igl/readPLY.h>
#include <igl/readTGF.h>

#include <Eigen/Geometry>
#include <cxxopts.hpp>
#include <iostream>

using ColorChannel =
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>;

int main(int argc, char* argv[]) {
  cxxopts::Options options("skinning", "Skinning on deformed mesh");
  auto options_adder = options.add_options();
  options_adder("help", "Print help");
  options_adder("m,mesh", "Mesh path (.ply)", cxxopts::value<std::string>());
  options_adder("t,texture", "Texture path (.png)",
                cxxopts::value<std::string>());
  options_adder("s,skeleton", "Skeleton path (.tgf)",
                cxxopts::value<std::string>());
  options_adder("o,output", "Output as an image (.png)",
                cxxopts::value<std::string>());
  auto args = options.parse(argc, argv);

  if (args.count("help")) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  Eigen::MatrixXd V, N, UV;
  Eigen::MatrixXi F, E;
  igl::readPLY(args["mesh"].as<std::string>(), V, F, E, N, UV);

  ColorChannel R;
  ColorChannel G;
  ColorChannel B;
  ColorChannel A;
  igl::png::readPNG(args["texture"].as<std::string>(), R, G, B, A);

  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);
  viewer.data().set_normals(N);
  viewer.data().set_uv(UV);
  viewer.data().show_texture = true;
  viewer.data().show_lines = false;
  viewer.data().set_texture(R, G, B, A);
  // rotate z 90deg and then y 180deg
  viewer.core().trackball_angle =
      Eigen::Quaternionf(-sqrt(0.5), sqrt(0.5), 0, 0);

  Eigen::MatrixXd C;
  Eigen::MatrixXi BE;
  igl::readTGF(args["skeleton"].as<std::string>(), C, BE);

  Eigen::VectorXi P;
  igl::directed_edge_parents(BE, P);

  if (!args.count("output")) {
    const Eigen::RowVector3d edge_color(70. / 255., 252. / 255., 167. / 255.);
    viewer.data().set_edges(C, BE, edge_color);
    viewer.launch();
  } else {
    int width = 1034;
    int height = 1024;
    ColorChannel Ro(width, height);
    ColorChannel Go(width, height);
    ColorChannel Bo(width, height);
    ColorChannel Ao(width, height);

    viewer.launch_init(false, false, "", width, height, true);
    viewer.core().draw_buffer(viewer.data(), true, Ro, Go, Bo, Ao);
    igl::png::writePNG(Ro, Go, Bo, Ao, args["output"].as<std::string>());
    viewer.launch_shut();
  }

  return 0;
}
