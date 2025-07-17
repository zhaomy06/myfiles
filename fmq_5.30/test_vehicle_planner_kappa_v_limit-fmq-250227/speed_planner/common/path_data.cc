/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file path_data.cc
 **/

#include "speed_planner/common/path_data.h"

#include <algorithm>


namespace apollo {
namespace planning {

using apollo::common::PathPoint;

using apollo::common::SLPoint;


bool PathData::SetDiscretizedPath(DiscretizedPath path) {

  discretized_path_ = std::move(path);

  return true;
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

bool PathData::Empty() const {
  return discretized_path_.empty();
}

common::PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}


void PathData::Clear() {
  discretized_path_.clear();
  path_point_decision_guide_.clear();
  path_reference_.clear();
}




}  // namespace planning
}  // namespace apollo
