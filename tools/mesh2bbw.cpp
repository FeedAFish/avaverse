#include <algorithm>
#include <filesystem>
#include <iostream>

#include "Eigen/Core"
#include "cxxopts.hpp"
#include "igl/barycenter.h"
#include "igl/bbw.h"
#include "igl/boundary_conditions.h"
#include "igl/boundary_facets.h"
#include "igl/copyleft/cgal/remesh_self_intersections.h"
#include "igl/copyleft/tetgen/mesh_with_skeleton.h"
#include "igl/copyleft/tetgen/tetrahedralize.h"
#include "igl/normalize_row_sums.h"
#include "igl/readTGF.h"
#include "igl/read_triangle_mesh.h"
#include "igl/remove_unreferenced.h"
#include "igl/volume.h"
#include "igl/winding_number.h"
#include "igl/writeDMAT.h"

// https://github.com/libigl/libigl-examples/blob/master/skeleton-poser/example.cpp

namespace fs = std::filesystem;

bool clean_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                double thresh, Eigen::MatrixXd& CV, Eigen::MatrixXi& CF) {
  {
    Eigen::MatrixXi _1;
    Eigen::VectorXi _2;
    Eigen::VectorXi IM;

    igl::copyleft::cgal::remesh_self_intersections(V, F, {}, CV, CF, _1, _2,
                                                   IM);

    std::for_each(CF.data(), CF.data() + CF.size(),
                  [&IM](int& i) { i = IM(i); });

    Eigen::MatrixXd old_CV = CV;
    Eigen::MatrixXi old_CF = CF;
    igl::remove_unreferenced(old_CV, old_CF, CV, CF, IM);
  }

  Eigen::MatrixXd TV;
  Eigen::MatrixXi TT;

  {
    Eigen::MatrixXi _1;
    if (igl::copyleft::tetgen::tetrahedralize(CV, CF, "", TV, TT, _1)) {
      return false;
    }
  }

  {
    Eigen::MatrixXd BC;
    igl::barycenter(TV, TT, BC);

    Eigen::VectorXd W;
    igl::winding_number(V, F, BC, W);
    W = W.array().abs();

    const int count = (W.array() > thresh).cast<int>().sum();
    Eigen::MatrixXi CT(count, TT.cols());
    int c = 0;
    for (int t = 0; t < TT.rows(); t++) {
      if (W(t) > thresh) {
        CT.row(c++) = TT.row(t);
      }
    }
    if (c != count) {
      return false;
    }
    igl::boundary_facets(CT, CF);
  }
  return true;
}

bool compute_tet_mesh(const Eigen::MatrixXd& CV, const Eigen::MatrixXi& CF,
                      const Eigen::MatrixXd& C, const Eigen::MatrixXi& BE,
                      int samples_per_bone, Eigen::MatrixXd& TV,
                      Eigen::MatrixXi& TT) {
  {
    Eigen::MatrixXi _1;
    return igl::copyleft::tetgen::mesh_with_skeleton(
        CV, CF, C, {}, BE, {}, samples_per_bone, "", TV, TT, _1);
  }
}

bool compute_bbw(const Eigen::MatrixXd& V, const Eigen::MatrixXd& TV,
                 const Eigen::MatrixXi& TT, const Eigen::MatrixXd& C,
                 const Eigen::MatrixXi& BE, int max_iter, Eigen::MatrixXd& W) {
  Eigen::VectorXi b;
  Eigen::MatrixXd bc;
  if (!igl::boundary_conditions(TV, TT, C, {}, BE, {}, b, bc)) {
    return false;
  }

  igl::BBWData bbw_data;
  bbw_data.verbosity = 2;
  bbw_data.active_set_params.max_iter = max_iter;
  if (!igl::bbw(TV, TT, b, bc, bbw_data, W)) {
    return false;
  }

  igl::normalize_row_sums(W, W);
  W.conservativeResize(V.rows(), W.cols());
  return true;
}

int main(int argc, char* argv[]) {
  cxxopts::Options options("mesh2bbw", "Compute bbw from given mesh");
  auto options_adder = options.add_options();
  options_adder("help", "Print help");
  options_adder("m,mesh", "Mesh path", cxxopts::value<fs::path>());
  options_adder("s,skeleton", "Skeleton path (.tgf)",
                cxxopts::value<fs::path>());
  options_adder("t,thresh", "Thresh for clean_mesh",
                cxxopts::value<double>()->default_value("0.5"));
  options_adder("b,sample_per_bone", "Sample per bone for compute_tet_mesh",
                cxxopts::value<int>()->default_value("10"));
  options_adder("i,max_iter", "Max iteration for compute_bbw",
                cxxopts::value<int>()->default_value("1000"));
  options_adder("o,output", "Output path (.dmat)", cxxopts::value<fs::path>());
  auto args = options.parse(argc, argv);

  if (args.count("help")) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh(args["mesh"].as<fs::path>(), V, F);

  Eigen::MatrixXd C;
  Eigen::MatrixXi BE;
  igl::readTGF(args["skeleton"].as<fs::path>(), C, BE);

  Eigen::MatrixXd CV;
  Eigen::MatrixXi CF;
  if (!clean_mesh(V, F, args["thresh"].as<double>(), CV, CF)) {
    return 1;
  };

  Eigen::MatrixXd TV;
  Eigen::MatrixXi TT;
  if (!compute_tet_mesh(CV, CF, C, BE, args["sample_per_bone"].as<int>(), TV,
                        TT)) {
    return 1;
  };

  Eigen::MatrixXd W;
  if (!compute_bbw(V, TV, TT, C, BE, args["max_iter"].as<int>(), W)) {
    return 1;
  };

  igl::writeDMAT(args["output"].as<fs::path>(), W);
  return 0;
}
