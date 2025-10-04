// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2019 Pierre MOULON, Romuald Perrot.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
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
#include <memory>
#include <string>

namespace fs = std::filesystem;

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;

/// Compute corresponding features between a series of views:
/// - Load view images description (regions: features & descriptors)
/// - Compute putative local feature matches (descriptors matching)
int main( int argc, char** argv )
{
  CmdLine cmd;

  std::string  sSfM_Data_Filename;
  std::string  sOutputMatchesFilename = "";
  float        fDistRatio             = 0.8f;
  std::string  sPredefinedPairList    = "";

  //required
  cmd.add( make_option( 'i', sSfM_Data_Filename, "input_file" ) );
  cmd.add( make_option( 'o', sOutputMatchesFilename, "output_file" ) );
  cmd.add( make_option( 'p', sPredefinedPairList, "pair_list" ) );

  try
  {
    if ( argc == 1 )
      throw std::string( "Invalid command line parameter." );
    cmd.process( argc, argv );
  }
  catch ( const std::string& s )
  {
    OPENMVG_LOG_INFO
      << "Usage: " << argv[ 0 ] << '\n'
      << "[-i|--input_file]   A SfM_Data file\n"
      << "[-o|--output_file]  Output file where computed matches are stored\n"
      << "[-p|--pair_list]    Pairs list file\n";

    OPENMVG_LOG_INFO << s;
    return EXIT_FAILURE;
  }

  OPENMVG_LOG_INFO << " You called : "
            << "\n"
            << argv[ 0 ] << "\n"
            << "--input_file " << sSfM_Data_Filename << "\n"
            << "--output_file " << sOutputMatchesFilename << "\n"
            << "--pair_list " << sPredefinedPairList << "\n"
            << "Optional parameters:"
            << "\n";

  if ( sOutputMatchesFilename.empty() )
  {
    OPENMVG_LOG_ERROR << "No output file set.";
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Read SfM Scene (image view & intrinsics data)
  //---------------------------------------
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    OPENMVG_LOG_ERROR << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read.";
    return EXIT_FAILURE;
  }
  const std::string sMatchesDirectory = fs::path(sOutputMatchesFilename).parent_path().string();

  //---------------------------------------
  // Load SfM Scene regions
  //---------------------------------------
  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace openMVG::features;
  const std::string sImage_describer = (fs::path(sMatchesDirectory) / "image_describer.json").string();
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type)
  {
    OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // a. Compute putative descriptor matches
  //    - Descriptor matching (according user method choice)
  //    - Keep correspondences only if NearestNeighbor ratio is ok
  //---------------------------------------

  // Load the corresponding view regions
  // Default regions provider (load & store all regions in memory)
  std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();

  // Show the progress on the command line:
  system::LoggerProgress progress;

  if (!regions_provider->load(sfm_data, sMatchesDirectory, regions_type, &progress)) {
    OPENMVG_LOG_ERROR << "Cannot load view regions from: " << sMatchesDirectory << ".";
    return EXIT_FAILURE;
  }

  PairWiseMatches map_PutativeMatches;

  // Build some alias from SfM_Data Views data:
  // - List views as a vector of filenames & image sizes
  std::vector<std::string>               vec_fileNames;
  std::vector<std::pair<size_t, size_t>> vec_imagesSize;
  {
    vec_fileNames.reserve(sfm_data.GetViews().size());
    vec_imagesSize.reserve(sfm_data.GetViews().size());
    for (const auto view_it : sfm_data.GetViews())
    {
      const View * v = view_it.second.get();
      vec_fileNames.emplace_back((fs::path(sfm_data.s_root_path) / v->s_Img_path).string());
      vec_imagesSize.emplace_back(v->ui_width, v->ui_height);
    }
  }

  OPENMVG_LOG_INFO << " - PUTATIVE MATCHES - ";
  // Allocate the right Matcher according the Matching requested method
  std::unique_ptr<Matcher> collectionMatcher;
  if ( regions_type->IsScalar() )
  {
    OPENMVG_LOG_INFO << "Using FAST_CASCADE_HASHING_L2 matcher";
    collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions(fDistRatio));
  }
  else if (regions_type->IsBinary())
  {
    OPENMVG_LOG_INFO << "Using HNSWHAMMING matcher";
    collectionMatcher.reset(new Matcher_Regions(fDistRatio, HNSW_HAMMING));
  }
  
  // Perform the matching
  auto t0 = std::chrono::steady_clock::now();
  {
    // Predefined pair list should always be provided.
    Pair_Set pairs;
    if ( !loadPairs( sfm_data.GetViews().size(), sPredefinedPairList, pairs ) )
    {
      OPENMVG_LOG_ERROR << "Failed to load pairs from file: \"" << sPredefinedPairList << "\"";
      return EXIT_FAILURE;
    }
    OPENMVG_LOG_INFO << "Running matching on #pairs: " << pairs.size();
    // Photometric matching of putative pairs
    collectionMatcher->Match( regions_provider, pairs, map_PutativeMatches, &progress );

    //---------------------------------------
    //-- Export putative matches & pairs
    //---------------------------------------
    if ( !Save( map_PutativeMatches, std::string( sOutputMatchesFilename ) ) )
    {
      OPENMVG_LOG_ERROR
        << "Cannot save computed matches in: "
        << sOutputMatchesFilename;
      return EXIT_FAILURE;
    }
  }
  auto t1 = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(t1 - t0).count();

  OPENMVG_LOG_INFO << "Task (Regions Matching) done in (s): " << elapsed;

  OPENMVG_LOG_INFO << "#Putative pairs: " << map_PutativeMatches.size();

  return EXIT_SUCCESS;
}
