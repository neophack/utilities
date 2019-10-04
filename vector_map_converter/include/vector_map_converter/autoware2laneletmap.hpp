/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
#ifndef AUTOWAREMAP2LANELETMAP_HPP
#define AUTOWAREMAP2LANELETMAP_HPP

#include <typeinfo>
#include <lanelet2_core/primitives/Lanelet.h>

#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/multi/geometries/register/multi_point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <ros/ros.h>
#include <autoware_map/util.h>
#include <autoware_map/map_handler.hpp>

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <vector_map/vector_map.h>
#include <amathutils_lib/amathutils.hpp>

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/regulatory_elements/detection_area.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

void createLaneletMap(
  const autoware_map::AutowareMapHandler &awmap_handler,
  lanelet::LaneletMapPtr lanelet_map_ptr);
lanelet::Point3d toLaneletPoint(const autoware_map_msgs::Point& awmap_pt);
lanelet::Point3d toLaneletPoint(autoware_map::Waypoint& awmap_wp);
void createDetectionArea(const autoware_map::AutowareMapHandler &awmap, lanelet::LaneletMapPtr lanelet_map_ptr);

lanelet::LineString3d createStopLineBetweenPoints(const lanelet::ConstPoint3d& pt1, const lanelet::ConstPoint3d& pt2, double width = 1, double offset = 0.1);
void createStopLines( const autoware_map::AutowareMapHandler &awmap, lanelet::LaneletMapPtr lanelet_map_ptr);

//keep angles within (M_PI, -M_PI]
double addAngles(double angle1, double angle2);

#endif
