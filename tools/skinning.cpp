#include <algorithm>
#include <iostream>
#include <numbers>

#include "Eigen/Geometry"
#include "cxxopts.hpp"
#include "igl/deform_skeleton.h"
#include "igl/directed_edge_parents.h"
#include "igl/lbs_matrix.h"
#include "igl/opengl/glfw/Viewer.h"
#include "igl/png/readPNG.h"
#include "igl/png/writePNG.h"
#include "igl/readDMAT.h"
#include "igl/readPLY.h"
#include "igl/readTGF.h"
#include "unsupported/Eigen/EulerAngles"

using ColorChannel =
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>;

void compute_skeleton_unit_vector(const Eigen::MatrixXd& C,
                                  const Eigen::MatrixXi& BE,
                                  Eigen::MatrixXd& VU) {
  VU.resize(BE.rows(), 3);
  for (int i = 0; i < BE.rows(); ++i) {
    VU.row(i) = (C.row(BE(i, 0)) - C.row(BE(i, 1))).normalized();
  }
}

void compute_transformation_matrix(const Eigen::MatrixXd& C,
                                   const Eigen::MatrixXi& BE,
                                   const Eigen::MatrixXd& CD,
                                   const Eigen::MatrixXi& BED,
                                   Eigen::MatrixXd& T) {
  Eigen::MatrixXd VU, VUD;
  compute_skeleton_unit_vector(C, BE, VU);
  compute_skeleton_unit_vector(CD, BED, VUD);

  int dim = C.cols();
  T.resize(BE.rows() * (dim + 1), dim);
  T.setZero();
  for (int i = 0; i < BE.rows(); ++i) {
    Eigen::Vector3d A = VU.row(i);
    Eigen::Vector3d B = VUD.row(i);
    Eigen::MatrixXd axis = A.cross(B);

    const float cosA = A.dot(B);
    const float k = 1.0f / (1.0f + cosA);

    Eigen::Matrix3d R;
    R << (axis(0) * axis(0) * k) + cosA, (axis(1) * axis(0) * k) - axis(2),
        (axis(2) * axis(0) * k) + axis(1), (axis(0) * axis(1) * k) + axis(2),
        (axis(1) * axis(1) * k) + cosA, (axis(2) * axis(1) * k) - axis(0),
        (axis(0) * axis(2) * k) - axis(1), (axis(1) * axis(2) * k) + axis(0),
        (axis(2) * axis(2) * k) + cosA;

    T.block(i * (dim + 1), 0, dim, dim) = R.transpose();
    T.row((i + 1) * (dim + 1) - 1) =
        CD.row(BED(i, 0)) - C.row(BE(i, 0)) * R.transpose();
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
  compute_transformation_matrix(C, BE, CD, BED, T);
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
    Eigen::MatrixXd CT;
    Eigen::MatrixXi BET;
    igl::deform_skeleton(C, BE, T, CT, BET);

    const Eigen::RowVector3d edge_color(70. / 255., 252. / 255., 167. / 255.);
    viewer.data().set_edges(CT, BET, edge_color);
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
