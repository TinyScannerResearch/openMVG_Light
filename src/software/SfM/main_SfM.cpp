// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2021 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"

#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/types.hpp"

// SfM Engines
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM2.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerMaxPair.hpp"

#include "third_party/cmdLine/cmdLine.h"

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>

namespace fs = std::filesystem;

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;

enum class ESfMSceneInitializer
{
  INITIALIZE_EXISTING_POSES,
  INITIALIZE_MAX_PAIR,
  INITIALIZE_AUTO_PAIR
};

enum class ESfMEngine
{
  INCREMENTAL,
  INCREMENTALV2
};

bool StringToEnum
(
  const std::string & str,
  ESfMEngine & sfm_engine
)
{
  const std::map<std::string, ESfMEngine> string_to_enum_mapping =
  {
    {"INCREMENTAL", ESfMEngine::INCREMENTAL},
    {"INCREMENTALV2", ESfMEngine::INCREMENTALV2}
  };
  const auto it  = string_to_enum_mapping.find(str);
  if (it == string_to_enum_mapping.end())
    return false;
  sfm_engine = it->second;
  return true;
}

bool StringToEnum
(
  const std::string & str,
  ESfMSceneInitializer & scene_initializer
)
{
  const std::map<std::string, ESfMSceneInitializer> string_to_enum_mapping =
  {
    {"EXISTING_POSE", ESfMSceneInitializer::INITIALIZE_EXISTING_POSES},
    {"MAX_PAIR", ESfMSceneInitializer::INITIALIZE_MAX_PAIR},
    {"AUTO_PAIR", ESfMSceneInitializer::INITIALIZE_AUTO_PAIR}
  };
  const auto it  = string_to_enum_mapping.find(str);
  if (it == string_to_enum_mapping.end())
    return false;
  scene_initializer = it->second;
  return true;
}

/// From 2 given image filenames, find the two corresponding index in the View list
bool computeIndexFromImageNames(
  const SfM_Data & sfm_data,
  const std::pair<std::string,std::string>& initialPairName,
  Pair& initialPairIndex)
{
  if (initialPairName.first == initialPairName.second)
  {
    OPENMVG_LOG_ERROR << "Invalid image names. You cannot use the same image to initialize a pair.";
    return false;
  }

  initialPairIndex = {UndefinedIndexT, UndefinedIndexT};

  /// List views filenames and find the one that correspond to the user ones:
  for (Views::const_iterator it = sfm_data.GetViews().begin();
     it != sfm_data.GetViews().end(); ++it)
  {
    const View * v = it->second.get();
    const std::string filename = fs::path(v->s_Img_path).filename().string();
    if (filename == initialPairName.first)
    {
      initialPairIndex.first = v->id_view;
    }
    else
    {
      if (filename == initialPairName.second)
      {
        initialPairIndex.second = v->id_view;
      }
    }
  }
  return (initialPairIndex.first != UndefinedIndexT &&
      initialPairIndex.second != UndefinedIndexT);
}

int main(int argc, char **argv)
{
  OPENMVG_LOG_INFO
      << "\n-----------------------------------------------------------"
      << "\n Structure from Motion:"
      << "\n-----------------------------------------------------------";
  CmdLine cmd;

  // Common options:
  std::string
      filename_sfm_data,
      directory_match,
      directory_output,
      engine_name = "INCREMENTAL";

  bool b_use_motion_priors = false;

  // Incremental SfM options
  int triangulation_method = static_cast<int>(ETriangulationMethod::INVERSE_DEPTH_WEIGHTED_MIDPOINT);
  int user_camera_model = PINHOLE_CAMERA_RADIAL3;

  // SfM v2
  std::string sfm_initializer_method = "MAX_PAIR";

  // Common options
  cmd.add( make_option('i', filename_sfm_data, "input_file") );
  cmd.add( make_option('m', directory_match, "match_dir") );
  cmd.add( make_option('o', directory_output, "output_dir") );
  cmd.add( make_option('s', engine_name, "sfm_engine") );

  cmd.add( make_switch('P', "prior_usage") );

  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch (const std::string& s) {

    OPENMVG_LOG_INFO << "Usage: " << argv[0] << '\n'
      << "[Required]\n"
      << "[-i|--input_file] path to a SfM_Data scene\n"
      << "[-m|--match_dir] path to the matches that corresponds to the provided SfM_Data scene\n"
      << "[-o|--output_dir] path where the output data will be stored\n"
      << "[-s|--sfm_engine] Type of SfM Engine to use for the reconstruction\n"
      << "\t INCREMENTAL   : add image sequentially to a 2 view seed\n"
      << "\t INCREMENTALV2 : add image sequentially to a 2 or N view seed (experimental)\n"
      << "\n\n"
      << "[Optional parameters]\n"
      << "\n\n"
      << "[Common]\n"
      << "[-P|--prior_usage] Enable usage of motion priors (i.e GPS positions) (default: false)\n";

    OPENMVG_LOG_ERROR << s;
    return EXIT_FAILURE;
  }

  b_use_motion_priors = cmd.used('P');

  if ( !isValid(openMVG::cameras::EINTRINSIC(user_camera_model)) )  {
    OPENMVG_LOG_ERROR << "Invalid camera type";
    return EXIT_FAILURE;
  }

  const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options = cameras::Intrinsic_Parameter_Type::ADJUST_ALL;
  const sfm::Extrinsic_Parameter_Type extrinsic_refinement_options = sfm::Extrinsic_Parameter_Type::ADJUST_ALL;

  ESfMSceneInitializer scene_initializer_enum;
  if (!StringToEnum(sfm_initializer_method, scene_initializer_enum))
  {
    OPENMVG_LOG_ERROR << "Invalid input for the SfM initializer option";
    return EXIT_FAILURE;
  }

  ESfMEngine sfm_engine_type;
  if (!StringToEnum(engine_name, sfm_engine_type))
  {
    OPENMVG_LOG_ERROR << "Invalid input for the SfM Engine type";
    return EXIT_FAILURE;
  }

  if (directory_output.empty())  {
    OPENMVG_LOG_ERROR << "It is an invalid output directory";
    return EXIT_FAILURE;
  }

  // SfM related

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  const ESfM_Data sfm_data_loading_etypes =
      scene_initializer_enum == ESfMSceneInitializer::INITIALIZE_EXISTING_POSES ?
        ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS) : ESfM_Data(VIEWS|INTRINSICS);
  if (!Load(sfm_data, filename_sfm_data, sfm_data_loading_etypes)) {
    OPENMVG_LOG_ERROR << "The input SfM_Data file \""<< filename_sfm_data << "\" cannot be read.";
    return EXIT_FAILURE;
  }

  if (!fs::exists(directory_output) && fs::is_directory(directory_output))
  {
    if (!fs::create_directory(directory_output))
    {
      OPENMVG_LOG_ERROR << "Cannot create the output directory";
      return EXIT_FAILURE;
    }
  }

  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace openMVG::features;
  const std::string sImage_describer = (fs::path(directory_match) / "image_describer.json").string();
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type)
  {
    OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
    return EXIT_FAILURE;
  }

  // Features reading
  std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  if (!feats_provider->load(sfm_data, directory_match, regions_type)) {
    OPENMVG_LOG_ERROR << "Cannot load view corresponding features in directory: " << directory_match << ".";
    return EXIT_FAILURE;
  }
  // Matches reading
  std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
  if // Try to read the provided match filename or the default one (matches.f.txt/bin)
  (
  !(matches_provider->load(sfm_data, (fs::path(directory_match) / "matches.f.txt").string()) ||
      matches_provider->load(sfm_data, (fs::path(directory_match) / "matches.f.bin").string()) ||
      matches_provider->load(sfm_data, (fs::path(directory_match) / "matches.e.txt").string()) ||
      matches_provider->load(sfm_data, (fs::path(directory_match) / "matches.e.bin").string()))
      )
  {
    OPENMVG_LOG_ERROR << "Cannot load the match file.";
    return EXIT_FAILURE;
  }

  std::unique_ptr<SfMSceneInitializer> scene_initializer;
  switch(scene_initializer_enum)
  {
  case ESfMSceneInitializer::INITIALIZE_AUTO_PAIR:
    OPENMVG_LOG_ERROR << "Not yet implemented.";
    return EXIT_FAILURE;
    break;
  case ESfMSceneInitializer::INITIALIZE_MAX_PAIR:
    scene_initializer.reset(new SfMSceneInitializerMaxPair(sfm_data,
                                 feats_provider.get(),
                                 matches_provider.get()));
    break;
  case ESfMSceneInitializer::INITIALIZE_EXISTING_POSES:
    scene_initializer.reset(new SfMSceneInitializer(sfm_data,
                            feats_provider.get(),
                            matches_provider.get()));
    break;
  default:
    OPENMVG_LOG_ERROR << "Unknown SFM Scene initializer method";
    return EXIT_FAILURE;
  }
  if (!scene_initializer)
  {
    OPENMVG_LOG_ERROR << "Invalid scene initializer.";
    return EXIT_FAILURE;
  }


  std::unique_ptr<ReconstructionEngine> sfm_engine;
  switch (sfm_engine_type)
  {
  case ESfMEngine::INCREMENTAL:
  {
    SequentialSfMReconstructionEngine * engine =
        new SequentialSfMReconstructionEngine(
          sfm_data,
          directory_output,
          (fs::path(directory_output) / "Reconstruction_Report.html").string());

    // Configuration:
    engine->SetFeaturesProvider(feats_provider.get());
    engine->SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    engine->SetUnknownCameraType(EINTRINSIC(user_camera_model));
    engine->SetTriangulationMethod(static_cast<ETriangulationMethod>(triangulation_method));
    engine->SetResectionMethod(static_cast<resection::SolverType>(resection::SolverType::DEFAULT));

    sfm_engine.reset(engine);
  }
    break;
  case ESfMEngine::INCREMENTALV2:
  {
    SequentialSfMReconstructionEngine2 * engine =
        new SequentialSfMReconstructionEngine2(
          scene_initializer.get(),
          sfm_data,
          directory_output,
          (fs::path(directory_output) / "Reconstruction_Report.html").string());

    // Configuration:
    engine->SetFeaturesProvider(feats_provider.get());
    engine->SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    engine->SetTriangulationMethod(static_cast<ETriangulationMethod>(triangulation_method));
    engine->SetUnknownCameraType(EINTRINSIC(user_camera_model));
    engine->SetResectionMethod(static_cast<resection::SolverType>(resection::SolverType::DEFAULT));

    sfm_engine.reset(engine);
  }
    break;
  default:
    break;
  }
  if (!sfm_engine)
  {
    OPENMVG_LOG_ERROR << "Cannot create the requested SfM Engine.";
    return EXIT_FAILURE;
  }

  sfm_engine->Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
  sfm_engine->Set_Extrinsics_Refinement_Type(extrinsic_refinement_options);
  sfm_engine->Set_Use_Motion_Prior(b_use_motion_priors);

  //---------------------------------------
  // Sequential reconstruction process
  //---------------------------------------

  openMVG::system::Timer timer;

  if (sfm_engine->Process())
  {
    OPENMVG_LOG_INFO << " Total Sfm took (s): " << timer.elapsed();

    //-- Export to disk computed scene (data & viewable results)
    OPENMVG_LOG_INFO << "...Export SfM_Data to disk.";
    Save(sfm_engine->Get_SfM_Data(),
       (fs::path(directory_output) / "sfm_data.bin").string(),
       ESfM_Data(ALL));

    Save(sfm_engine->Get_SfM_Data(),
       (fs::path(directory_output) / "cloud_and_poses.ply").string(),
       ESfM_Data(ALL));

    return EXIT_SUCCESS;
  }
  return EXIT_FAILURE;
}
