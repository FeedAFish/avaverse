#include "avaverse/skinning.hpp"

#include <filesystem>
#include <stdexcept>

#include "Eigen/Geometry"
#include "avaverse/config.hpp"
#include "avaverse/type.hpp"
#include "igl/deform_skeleton.h"
#include "igl/lbs_matrix.h"
#include "igl/png/readPNG.h"
#include "igl/readDMAT.h"
#include "igl/readPLY.h"
#include "igl/readTGF.h"

namespace avaverse {

Skinning::Skinning(Eigen::MatrixXd&& V, Eigen::MatrixXi&& F,
                   Eigen::MatrixXd&& UV, ColorChannel&& R, ColorChannel&& G,
                   ColorChannel&& B, ColorChannel&& A, Skeleton&& C,
                   Eigen::MatrixXd&& M)
    : V(V), F(F), UV(UV), R(R), G(G), B(B), A(A), C(C), M(M) {
  viewer_.data().set_mesh(V, F);
  viewer_.data().set_uv(UV);
  viewer_.data().show_texture = true;
  viewer_.data().show_lines = false;
  viewer_.data().set_texture(R, G, B, A);
  // rotate z 90deg and then rotate y 180deg
  viewer_.core().trackball_angle =
      Eigen::Quaternionf(-sqrt(0.5), sqrt(0.5), 0, 0);
}

Skinning::~Skinning() { viewer_.launch_shut(); }

Skinning Skinning::from_igl_path(const fs::path& mesh_path,
                                 const fs::path& texture_path,
                                 const fs::path& skeleton_path,
                                 const fs::path& weight_path) {
  Eigen::MatrixXd V;
  Eigen::MatrixXd UV;
  Eigen::MatrixXi F;
  {
    Eigen::MatrixXd _1, _3, _5, _7;
    Eigen::MatrixXi _2;
    std::vector<std::string> _4, _6, _8, _9;
    igl::readPLY(mesh_path, V, F, _2, _1, UV, _3, _4, _5, _6, _7, _8, _9);
  }

  ColorChannel R;
  ColorChannel G;
  ColorChannel B;
  ColorChannel A;
  igl::png::readPNG(texture_path, R, G, B, A);

  Skeleton C;
  {
    Eigen::MatrixXd _C;
    Eigen::MatrixXi _BE;
    igl::readTGF(skeleton_path, _C, _BE);
    check_skeleton_structure(_C, _BE);
    C = _C;
  }

  Eigen::MatrixXd W, M;
  igl::readDMAT(weight_path, W);
  igl::lbs_matrix(V, W, M);

  return Skinning(std::move(V), std::move(F), std::move(UV), std::move(R),
                  std::move(G), std::move(B), std::move(A), std::move(C),
                  std::move(M));
}

void Skinning::launch(bool with_gui, int width, int height) {
  if (with_gui) {
    viewer_.launch_init(true, false, "skinning viewer", width, height);
    viewer_.launch_rendering(true);
  } else {
    viewer_.launch_init(false, false, "", width, height, true);
  }
}

void Skinning::deform(const fs::path& deform_skeleton_path) {
  Eigen::MatrixXd _C;
  Eigen::MatrixXi _BE;
  igl::readTGF(deform_skeleton_path, _C, _BE);
  check_skeleton_structure(_C, _BE);
  deform(_C);
}

void Skinning::deform(const Skeleton& CD) {
  for (int i = 0; i < BE.rows(); ++i) {
    const auto& A1 = C.row(BE(i, 0));
    const auto& B1 = C.row(BE(i, 1));
    const auto& A2 = CD.row(BE(i, 0));
    const auto& B2 = CD.row(BE(i, 1));
    const auto& axis = ((A1 - B1).normalized()).cross((A2 - B2).normalized());
    const auto& cos = ((A1 - B1).normalized()).dot((A2 - B2).normalized());
    const auto& k = 1 / (1 + cos);

    // https://gist.github.com/kevinmoran/b45980723e53edeb8a5a43c49f134724

    T_(i * (kDim + 1), 0) = (axis(0) * axis(0) * k) + cos;
    T_(i * (kDim + 1), 1) = (axis(0) * axis(1) * k) + axis(2);
    T_(i * (kDim + 1), 2) = (axis(0) * axis(2) * k) - axis(1);

    T_(i * (kDim + 1) + 1, 0) = (axis(1) * axis(0) * k) - axis(2);
    T_(i * (kDim + 1) + 1, 1) = (axis(1) * axis(1) * k) + cos;
    T_(i * (kDim + 1) + 1, 2) = (axis(1) * axis(2) * k) + axis(0);

    T_(i * (kDim + 1) + 2, 0) = (axis(2) * axis(0) * k) + axis(1);
    T_(i * (kDim + 1) + 2, 1) = (axis(2) * axis(1) * k) - axis(0);
    T_(i * (kDim + 1) + 2, 2) = (axis(2) * axis(2) * k) + cos;

    if (P(i) == -1) {
      T_.row((i + 1) * (kDim + 1) - 1) =
          A2 - A1 * T_.block(i * (kDim + 1), 0, kDim, kDim);
    } else {
      T_.row((i + 1) * (kDim + 1) - 1) =
          C.row(BE(P(i), 1)) * T_.block(P(i) * (kDim + 1), 0, kDim, kDim) +
          T_.row((P(i) + 1) * (kDim + 1) - 1) -
          A1 * T_.block(i * (kDim + 1), 0, kDim, kDim);
    }
  }
  Eigen::MatrixXd U = M * T_;
  viewer_.data().set_vertices(U);
}

void Skinning::add_edges(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E,
                         int r, int g, int b) {
  const Eigen::RowVector3d color = Eigen::RowVector3d(r, g, b) / 255;
  add_edges(V, E, color);
}

void Skinning::add_edges(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E,
                         const Eigen::RowVector3d& color) {
  Eigen::MatrixXd P1(E.rows(), V.cols());
  Eigen::MatrixXd P2(E.rows(), V.cols());

  for (int i = 0; i < E.rows(); ++i) {
    P1.row(i) = V.row(E(i, 0));
    P2.row(i) = V.row(E(i, 1));
  }

  viewer_.data().add_edges(P1, P2, color);
}

void Skinning::show_skeleton(int r, int g, int b) {
  const Eigen::RowVector3d color = Eigen::RowVector3d(r, g, b) / 255;
  show_skeleton(color);
}

void Skinning::show_skeleton(const Eigen::RowVector3d& color) {
  Eigen::MatrixXd CT;
  Eigen::MatrixXi BET;
  igl::deform_skeleton(C, BE, T_, CT, BET);

  add_edges(CT, BET, color);
}

void Skinning::draw_buffer(ColorChannel& R, ColorChannel& G, ColorChannel& B,
                           ColorChannel& A) {
  viewer_.core().draw_buffer(viewer_.data(), true, R, G, B, A);
}

void Skinning::draw_image(EigenImage& I) {
  viewer_.core().draw_buffer(viewer_.data(), true, I);
}

void Skinning::check_skeleton_structure(const Eigen::MatrixXd& C,
                                        const Eigen::MatrixXi& BE) {
  if ((BE.array() != Skinning::BE.array()).any() || C.rows() != kNumPose ||
      C.cols() != kDim) {
    throw std::invalid_argument("wrong skeleton structure");
  }
}

};  // namespace avaverse
