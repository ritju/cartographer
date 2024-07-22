/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {

namespace mapping {
class TrajectoryBuilderInterface;
}

namespace sensor {

class Data {
 public:
  explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
  virtual ~Data() {}

  virtual common::Time GetTime() const = 0;
  const std::string &GetSensorId() const { return sensor_id_; }
  virtual void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *trajectory_builder) = 0;
  virtual void SetLocalizationScore(float localization_score, std::vector<float> corrected_submap_pose, float global_pose_x, float global_pose_y){
    corrected_submap_pose_ = corrected_submap_pose;
    localization_score_ = localization_score;
    global_pose_x_ = global_pose_x;
    global_pose_y_ = global_pose_y;
  }

 protected:
  const std::string sensor_id_;
  float localization_score_;
  std::vector<float> corrected_submap_pose_;
  float global_pose_x_, global_pose_y_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DATA_H_
