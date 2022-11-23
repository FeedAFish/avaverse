#include "avaverse/skinning.hpp"

#include <filesystem>
#include <iostream>

#include "cxxopts.hpp"
#include "igl/png/writePNG.h"

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  cxxopts::Options options("skinning", "Skinning on deformed mesh");
  auto options_adder = options.add_options();
  options_adder("help", "Print help");
  options_adder("m,mesh", "Mesh path (.ply)", cxxopts::value<fs::path>());
  options_adder("t,texture", "Texture path (.png)", cxxopts::value<fs::path>());
  options_adder("s,skeleton", "Skeleton path (.tgf)",
                cxxopts::value<fs::path>());
  options_adder("w,weight", "Weight path (.dmat)", cxxopts::value<fs::path>());
  options_adder("d,deform", "Deform skeleton path (.tgf)",
                cxxopts::value<fs::path>());
  options_adder("o,output", "Output as an image (.png)",
                cxxopts::value<fs::path>());
  auto args = options.parse(argc, argv);

  if (args.count("help")) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  auto skinning = avaverse::Skinning::from_igl_path(
      args["mesh"].as<fs::path>(), args["texture"].as<fs::path>(),
      args["skeleton"].as<fs::path>(), args["weight"].as<fs::path>());
  skinning.deform(args["deform"].as<fs::path>());

  if (!args.count("output")) {
    skinning.add_edges(skinning.C, skinning.BE, 70, 252, 167);
    skinning.show_skeleton(255, 0, 0);
    skinning.launch(true);
  } else {
    skinning.launch(false);
    avaverse::ColorChannel R;
    avaverse::ColorChannel G;
    avaverse::ColorChannel B;
    avaverse::ColorChannel A;
    skinning.draw_buffer(R, G, B, A);
    igl::png::writePNG(R, G, B, A, args["output"].as<fs::path>());
  }

  return 0;
}
