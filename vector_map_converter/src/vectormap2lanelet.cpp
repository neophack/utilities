/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector_map/vector_map.h>

#include <vector_map_converter/vectormap2lanelet.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vectormap2lanelet2");
  ros::NodeHandle nh;

  vector_map::VectorMap vmap;
  vector_map::Category category = vector_map::Category::ALL;
  vmap.subscribe(nh, category, ros::Duration(2));

  lanelet::LaneletMapPtr lmap(new lanelet::LaneletMap);

  convertVectorMap2Lanelet2(vmap, lmap);
  lanelet::projection::UtmProjector projector(lanelet::Origin({ 0.0, 0.0 }));

  lanelet::write("lanelet_map.osm", *lmap, projector);

  return EXIT_SUCCESS;
}
