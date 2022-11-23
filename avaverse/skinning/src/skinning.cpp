#include "avaverse/skinning.hpp"

#include <filesystem>
#include <stdexcept>

#include "Eigen/Geometry"
#include "avaverse/config.hpp"
#include "avaverse/type.hpp"
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
    Eigen::MatrixXd _1;
    Eigen::MatrixXi _2;
    igl::readPLY(mesh_path, V, F, _2, _1, UV);
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
    viewer_.launch(true, false, "skinning viewer", width, height);
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

    T_.row((i + 1) * (kDim + 1) - 1) =
        A2 - A1 * T_.block(i * (kDim + 1), 0, kDim, kDim);
  }
  Eigen::MatrixXd U = M * T_;
  viewer_.data().set_vertices(U);
}

void Skinning::check_skeleton_structure(const Eigen::MatrixXd& C,
                                        const Eigen::MatrixXi& BE) {
  if ((BE.array() != Skinning::BE.array()).any() || C.rows() != kNumPose ||
      C.cols() != kDim) {
    throw std::invalid_argument("wrong skeleton structure");
  }
}

};  // namespace avaverse
