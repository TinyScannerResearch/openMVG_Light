// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/matcher_hnsw.hpp"
#include "openMVG/matching/metric.hpp"
#include "openMVG/matching/metric_hamming.hpp"
#include "openMVG/system/logger.hpp"

namespace openMVG {
namespace matching {

void Match
(
  const matching::EMatcherType & matcher_type,
  const features::Regions & database_regions,
  const features::Regions & query_regions,
  matching::IndMatches & matches
)
{
  const std::unique_ptr<RegionsMatcher> matcher =
    RegionMatcherFactory(matcher_type, database_regions);
  if (matcher)
  {
    matcher->Match(query_regions, matches);
  }
}

void DistanceRatioMatch
(
  float f_dist_ratio,
  const matching::EMatcherType & matcher_type,
  const features::Regions & database_regions,
  const features::Regions & query_regions,
  matching::IndMatches & matches
)
{
  const std::unique_ptr<RegionsMatcher> matcher =
    RegionMatcherFactory(matcher_type, database_regions);
  if (matcher)
  {
    matcher->MatchDistanceRatio(f_dist_ratio, query_regions, matches);
  }
}

std::unique_ptr<RegionsMatcher> RegionMatcherFactory
(
  matching::EMatcherType eMatcherType,
  const features::Regions & regions
)
{

  std::unique_ptr<RegionsMatcher> region_matcher;
  // Switch regions type ID, matcher & Metric: initialize the Matcher interface
  if (regions.IsBinary() && regions.Type_id() == typeid(unsigned char).name())
  {
    switch (eMatcherType)
    {
      case HNSW_HAMMING:
      {
        using MetricT = Hamming<unsigned char>;
        using MatcherT = HNSWMatcher<unsigned char, MetricT, HNSWMETRIC::HAMMING_HNSW>;
        region_matcher.reset(new matching::RegionsMatcherT<MatcherT>(regions, false));
      }
      break;
      default:
          OPENMVG_LOG_ERROR << "Using unknown matcher type";
    }
  }
  else
  {
    OPENMVG_LOG_ERROR << "Please consider add this region type_id to Matcher_Regions_Database::Match(...)\n"
      << "typeid: " << regions.Type_id();
  }
  return region_matcher;
}

}  // namespace matching
}  // namespace openMVG
