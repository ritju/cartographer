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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data {
 public:
  Dispatchable(const std::string &sensor_id, const DataType &data)
      : Data(sensor_id), data_(data), localization_score_(0), corrected_submap_pose_(10) {}

  common::Time GetTime() const override { return data_.time; }
  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *const trajectory_builder) override {
    trajectory_builder->SetLocalizationScore(localization_score_, corrected_submap_pose_, global_pose_x_, global_pose_y_);
    trajectory_builder->AddSensorData(sensor_id_, data_);

  }
  const DataType &data() const { return data_; }
  void SetLocalizationScore(float localization_score, std::vector<float> corrected_submap_pose, const float global_pose_x, const float global_pose_y){
    corrected_submap_pose_ = corrected_submap_pose;
    localization_score_ = localization_score;
    global_pose_x_ = global_pose_x;
    global_pose_y_ = global_pose_y;
    // LOG(INFO) << "Dispatchable::localization_score_" << localization_score_;
  }

 private:
  const DataType data_;
  float localization_score_;
  std::vector<float> corrected_submap_pose_;
  float global_pose_x_, global_pose_y_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
  return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
