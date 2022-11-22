#include <igl/directed_edge_parents.h>
#include <igl/lbs_matrix.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/png/readPNG.h>
#include <igl/png/writePNG.h>
#include <igl/readDMAT.h>
#include <igl/readPLY.h>
#include <igl/readTGF.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cxxopts.hpp>
#include <iostream>
#include <numbers>
#include <unsupported/Eigen/EulerAngles>

using ColorChannel =
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>;

void compute_angle_of_skeleton(const Eigen::MatrixXd& skeleton,
                               Eigen::MatrixXd& angles) {
  angles = skeleton;
  angles = angles.rowwise().normalized().array().acos().matrix();
}

void compute_transformation_matrix(const Eigen::MatrixXd& skeleton,
                                   const Eigen::MatrixXd& deform,
                                   Eigen::MatrixXd& T) {
  Eigen::MatrixXd angles, anglesD;
  compute_angle_of_skeleton(skeleton, angles);
  compute_angle_of_skeleton(deform, anglesD);
  Eigen::MatrixXd rotate_angles = (anglesD - angles) * 180 / std::numbers::pi;

  int dim = rotate_angles.cols();
  T.resize((dim + 1) * rotate_angles.rows(), dim);
  for (int i = 0; i < rotate_angles.rows(); ++i) {
    Eigen::EulerAnglesXYZd euler(rotate_angles(i, 0), rotate_angles(i, 1),
                                 rotate_angles(i, 2));
    Eigen::Affine3d a = Eigen::Affine3d::Identity();
    T.block(i * (dim + 1), 0, dim + 1, dim) =
        a.matrix().transpose().block(0, 0, dim + 1, dim);
  }
}

int main(int argc, char* argv[]) {
  cxxopts::Options options("skinning", "Skinning on deformed mesh");
  auto options_adder = options.add_options();
  options_adder("help", "Print help");
  options_adder("m,mesh", "Mesh path (.ply)", cxxopts::value<std::string>());
  options_adder("t,texture", "Texture path (.png)",
                cxxopts::value<std::string>());
  options_adder("s,skeleton", "Skeleton path (.tgf)",
                cxxopts::value<std::string>());
  options_adder("w,weight", "Weight path (.dmat)",
                cxxopts::value<std::string>());
  options_adder("d,deform", "Deform skeleton path (.tgf)",
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

  Eigen::MatrixXd C;
  Eigen::MatrixXi BE;
  igl::readTGF(args["skeleton"].as<std::string>(), C, BE);

  Eigen::VectorXi P;
  igl::directed_edge_parents(BE, P);

  Eigen::MatrixXd CD;
  Eigen::MatrixXi BED;
  igl::readTGF(args["deform"].as<std::string>(), CD, BED);

  Eigen::MatrixXd W, M;
  igl::readDMAT(args["weight"].as<std::string>(), W);
  igl::lbs_matrix(V, W, M);

  Eigen::MatrixXd T;
  compute_transformation_matrix(C, CD, T);
  Eigen::MatrixXd U = M * T;

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

  viewer.data().set_vertices(U);
  viewer.data().compute_normals();

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
