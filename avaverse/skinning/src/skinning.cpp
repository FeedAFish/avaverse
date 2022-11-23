#include "skinning/skinning.hpp"

#include <filesystem>
#include <stdexcept>

#include "common/type.hpp"
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
    : V(V), F(F), UV(UV), R(R), G(G), B(B), A(A), C(C), M(M) {}

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

void Skinning::check_skeleton_structure(const Eigen::MatrixXd& C,
                                        const Eigen::MatrixXi& BE) {
  if ((BE.array() != Skinning::BE.array()).any() || C.rows() != kNumPose ||
      C.cols() != 3) {
    throw std::invalid_argument("wrong skeleton structure");
  }
}

};  // namespace avaverse
