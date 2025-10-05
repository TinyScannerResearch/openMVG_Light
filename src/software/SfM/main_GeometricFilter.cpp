// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2019 Pierre MOULON, Romuald PERROT

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "third_party/cmdLine/cmdLine.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <locale>
#include <memory>
#include <string>

namespace fs = std::filesystem;

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;

/// Compute corresponding features between a series of views:
/// - Load view images description (regions: features & descriptors)
/// - Compute putative local feature matches (descriptors matching)
/// - Compute geometric coherent feature matches (robust model estimation from putative matches)
/// - Export computed data
int main( int argc, char** argv )
{
  CmdLine cmd;

  // The scene
  std::string sSfM_Data_Filename;
  // The input matches
  std::string sPutativeMatchesFilename;
  // The output matches
  std::string sFilteredMatchesFilename;

  bool bGuided_matching  = false;
  int  imax_iteration    = 2048;

  //required
  cmd.add( make_option( 'i', sSfM_Data_Filename, "input_file" ) );
  cmd.add( make_option( 'o', sFilteredMatchesFilename, "output_file" ) );
  cmd.add( make_option( 'm', sPutativeMatchesFilename, "matches" ) );

  try
  {
    if ( argc == 1 )
      throw std::string( "Invalid command line parameter." );
    cmd.process( argc, argv );
  }
  catch ( const std::string& s )
  {
    OPENMVG_LOG_INFO << "Usage: " << argv[0] << '\n'
                     << "[-i|--input_file]       A SfM_Data file\n"
                     << "[-m|--matches]          (Input) matches filename\n"
                     << "[-o|--output_file]      (Output) filtered matches filename\n";

    OPENMVG_LOG_INFO << s;
    return EXIT_FAILURE;
  }

  OPENMVG_LOG_INFO << " You called : "
                   << "\n"
                   << argv[0] << "\n"
                   << "--input_file:        " << sSfM_Data_Filename << "\n"
                   << "--matches:           " << sPutativeMatchesFilename << "\n"
                   << "--output_file:       " << sFilteredMatchesFilename << "\n";

  if ( sFilteredMatchesFilename.empty() )
  {
    OPENMVG_LOG_ERROR << "It is an invalid output file";
    return EXIT_FAILURE;
  }
  if ( sSfM_Data_Filename.empty() )
  {
    OPENMVG_LOG_ERROR << "It is an invalid SfM file";
    return EXIT_FAILURE;
  }
  if ( sPutativeMatchesFilename.empty() )
  {
    OPENMVG_LOG_ERROR << "It is an invalid putative matche file";
    return EXIT_FAILURE;
  }

  const std::string sMatchesDirectory = fs::path(sPutativeMatchesFilename).parent_path().string();

  //---------------------------------------
  // Read SfM Scene (image view & intrinsics data)
  //---------------------------------------
  SfM_Data sfm_data;
  if ( !Load( sfm_data, sSfM_Data_Filename, ESfM_Data( VIEWS | INTRINSICS ) ) )
  {
    OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read.";
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Load SfM Scene regions
  //---------------------------------------
  using namespace openMVG::features;
  // Consider that the image_describer.json is inside the matches directory (which is bellow the sfm_data.bin)
  const std::string        sImage_describer = (fs::path(sMatchesDirectory) / "image_describer.json").string();
  std::unique_ptr<Regions> regions_type     = Init_region_type_from_file( sImage_describer );
  if ( !regions_type )
  {
    OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
    return EXIT_FAILURE;
  }

  // Load the corresponding view regions
  // Default regions provider (load & store all regions in memory)
  std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();

  // Show the progress on the command line:
  system::LoggerProgress progress;
  if ( !regions_provider->load( sfm_data, sMatchesDirectory, regions_type, &progress ) )
  {
    OPENMVG_LOG_ERROR << "Invalid regions.";
    return EXIT_FAILURE;
  }

  PairWiseMatches map_PutativeMatches;
  //---------------------------------------
  // A. Load initial matches
  //---------------------------------------
  if ( !Load( map_PutativeMatches, sPutativeMatchesFilename ) )
  {
    OPENMVG_LOG_ERROR << "Failed to load the initial matches file.";
    return EXIT_FAILURE;
  }

  std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
      new ImageCollectionGeometricFilter( &sfm_data, regions_provider ) );

  if ( filter_ptr )
  {
    auto t0 = std::chrono::steady_clock::now();
    const double  d_distance_ratio = 0.6;

    PairWiseMatches map_GeometricMatches;
    filter_ptr->Robust_model_estimation(
        GeometricFilter_FMatrix_AC( 4.0, imax_iteration ),
        map_PutativeMatches,
        bGuided_matching,
        d_distance_ratio,
        &progress );
    map_GeometricMatches = filter_ptr->Get_geometric_matches();

    //---------------------------------------
    //-- Export geometric filtered matches
    //---------------------------------------
    if ( !Save( map_GeometricMatches, sFilteredMatchesFilename ) )
    {
      OPENMVG_LOG_ERROR << "Cannot save filtered matches in: " << sFilteredMatchesFilename;
      return EXIT_FAILURE;
    }

    auto t1 = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(t1 - t0).count();

    OPENMVG_LOG_INFO << "Task done in (s): " << elapsed;
  }
  return EXIT_SUCCESS;
}
