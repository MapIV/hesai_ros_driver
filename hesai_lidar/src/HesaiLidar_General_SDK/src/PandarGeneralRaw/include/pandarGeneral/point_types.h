/******************************************************************************
 * Copyright 2018 The Hesai Technology Authors. All Rights Reserved.
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

#ifndef INCLUDE_POINT_TYPES_H_
#define INCLUDE_POINT_TYPES_H_

#include <pcl/point_types.h>

struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  int8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIR,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRADT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
       (std::uint16_t, ring, ring)(float, azimuth, azimuth)
       (float, distance, distance)(int8_t, return_type, return_type)
       (double, time_stamp, time_stamp))

typedef PointXYZIRADT PPointXYZIRADT;
typedef pcl::PointCloud<PPointXYZIRADT> PPointCloudXYZIRADT;

typedef PointXYZIR PPointXYZIR;
typedef pcl::PointCloud<PPointXYZIR> PPointCloudXYZIR;

#endif  // INCLUDE_POINT_TYPES_H_
