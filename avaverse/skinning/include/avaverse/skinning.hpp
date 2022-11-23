#pragma once

#include <filesystem>
#include <string>

#include "Eigen/Core"
#include "avaverse/config.hpp"
#include "avaverse/type.hpp"
#include "igl/opengl/glfw/Viewer.h"

namespace avaverse {

namespace fs = std::filesystem;

using TransformationMatrix =
    Eigen::Matrix<double, 14 /* BE.rows() */ * (kDim + 1), kDim>;

class Skinning {
 public:
  static Skinning from_igl_path(const fs::path& mesh_path,
                                const fs::path& texture_path,
                                const fs::path& skeleton_path,
                                const fs::path& weight_path);

  ~Skinning();

  inline const static auto BE = Eigen::Matrix<int, 14, 2>({{0, 1},
                                                           {1, 2},
                                                           {1, 5},
                                                           {1, 8},
                                                           {2, 3},
                                                           {3, 4},
                                                           {5, 6},
                                                           {6, 7},
                                                           {8, 9},
                                                           {8, 12},
                                                           {9, 10},
                                                           {10, 11},
                                                           {12, 13},
                                                           {13, 14}});

  void launch(bool with_gui, int width = 0, int height = 0);

  void deform(const fs::path& deform_skeleton_path);
  void deform(const Skeleton& CD);

  void add_edges(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E, int r,
                 int g, int b);
  void add_edges(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E,
                 const Eigen::RowVector3d& color);

  void show_skeleton(int r, int g, int b);
  void show_skeleton(const Eigen::RowVector3d& color);

  const Eigen::MatrixXd V;
  const Eigen::MatrixXi F;
  const Eigen::MatrixXd UV;
  const ColorChannel R;
  const ColorChannel G;
  const ColorChannel B;
  const ColorChannel A;
  const Skeleton C;
  const Eigen::MatrixXd M;

 private:
  Skinning(Eigen::MatrixXd&& V, Eigen::MatrixXi&& F, Eigen::MatrixXd&& UV,
           ColorChannel&& R, ColorChannel&& G, ColorChannel&& B,
           ColorChannel&& A, Skeleton&& C, Eigen::MatrixXd&& M);

  static void check_skeleton_structure(const Eigen::MatrixXd& C,
                                       const Eigen::MatrixXi& BE);

  igl::opengl::glfw::Viewer viewer_;

  TransformationMatrix T_ = TransformationMatrix::Zero();
};

};  // namespace avaverse
