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
 #include <vector_map_converter/autoware2laneletmap.hpp>
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

using lanelet::utils::getId;

void createLaneletMap(
  const autoware_map::AutowareMapHandler &awmap_handler,
  lanelet::LaneletMapPtr lanelet_map_ptr)
{
  createStopLines(awmap_handler, lanelet_map_ptr);
  createDetectionArea(awmap_handler, lanelet_map_ptr);
}

lanelet::Point3d toLaneletPoint(const autoware_map_msgs::Point& awmap_pt)
{
  lanelet::Point3d lanelet_pt(getId());
  lanelet_pt.x() = awmap_pt.x;
  lanelet_pt.y() = awmap_pt.y;
  lanelet_pt.z() = awmap_pt.z;
  return lanelet_pt;
}

lanelet::Point3d toLaneletPoint(autoware_map::Waypoint& awmap_wp)
{
  auto awmap_pt = awmap_wp.getPointPtr();
  lanelet::Point3d lanelet_pt(getId());
  lanelet_pt.x() = awmap_pt->x;
  lanelet_pt.y() = awmap_pt->y;
  lanelet_pt.z() = awmap_pt->z;
  return lanelet_pt;
}


void createDetectionArea(const autoware_map::AutowareMapHandler &awmap, lanelet::LaneletMapPtr lanelet_map_ptr)
{
  std::map<int, lanelet::Polygon3d> polygon_map;

  for( auto awmap_lane : awmap.findByFilter([] (const autoware_map::Lane l){return true; }) )
  {

    lanelet::Polygons3d polygons;
    for( auto awmap_relation : awmap.findByFilter([&](const autoware_map::LaneAttributeRelation lar){ return (lar.lane_id == awmap_lane.lane_id && lar.attribute_type == autoware_map_msgs::LaneAttributeRelation::CROSS_WALK); }) )
    {
      // add the area if it is already made. Otherwise, create and then add.
      if(polygon_map.find(awmap_relation.area_id) != polygon_map.end())
      {
        polygons.push_back(polygon_map.at(awmap_relation.area_id));
      }
      else
      {
        lanelet::Points3d polygon_points;

        auto awmap_area = awmap.findById<autoware_map::Area>(awmap_relation.area_id);
        for(auto point : awmap_area.getPoints())
        {
          lanelet::Point3d lanelet_pt = toLaneletPoint(*point);
          polygon_points.push_back(lanelet_pt);
        }
        polygon_points.push_back(polygon_points.front());
        
        // lanelet::LineString3d linestring(getId(), polygon_points);
        lanelet::Polygon3d polygon(getId(), polygon_points);
        polygon.attributes()["type"] = "detection_area";
        
        polygons.push_back(polygon);
        polygon_map[awmap_relation.area_id] = polygon;

      }
    }  // LaneAttributeRelation Loop
    if(polygons.empty()) continue;
    
    // stopline
    auto waypoints = awmap_lane.getWaypoints();
    auto pt1 = toLaneletPoint(*waypoints.at(0));
    auto pt2 = toLaneletPoint(*waypoints.at(1));
    double width = 2.0;
    lanelet::LineString3d stopline = createStopLineBetweenPoints(pt1, pt2, width);

    auto detection_area = lanelet::autoware::DetectionArea::make(getId(), lanelet::AttributeMap(), polygons, stopline);
    lanelet_map_ptr->add(detection_area);

    
  }  // Lane loop
}

lanelet::LineString3d createStopLineBetweenPoints(const lanelet::ConstPoint3d& pt1, const lanelet::ConstPoint3d& pt2, double width, double offset)
{
  double yaw = atan2(pt2.y() - pt1.y(), pt2.x() - pt1.x());
  std::cout << pt2 << std::endl << pt1 << std::endl << yaw << std::endl;
  double angle_left, angle_right;
  angle_left = addAngles(yaw, M_PI / 2);
  angle_right = addAngles(yaw, -M_PI / 2);

  double r = width / 2;

  //stop line cannot be right on waypoint with current decision_maker
  double epsilon_x = cos(yaw) * offset;
  double epsilon_y = sin(yaw) * offset;

  lanelet::Point3d start_point(getId());
  start_point.x() = pt1.x() + r * cos(angle_left) + epsilon_x;
  start_point.y() = pt1.y() + r * sin(angle_left) + epsilon_y;
  start_point.z() = pt1.z();


  lanelet::Point3d end_point(getId());
  end_point.x() = pt1.x() + r * cos(angle_right) + epsilon_x;
  end_point.y() = pt1.y() + r * sin(angle_right) + epsilon_y;
  end_point.z() = pt1.z();

  lanelet::LineString3d stopline = lanelet::LineString3d(getId(), {start_point, end_point});
  stopline.attributes()["type"] = "stop_line";

  return stopline;
}


void createStopLines( const autoware_map::AutowareMapHandler &awmap,
                      lanelet::LaneletMapPtr lanelet_map_ptr)
{

  for(auto wp : awmap.findByFilter([] (const autoware_map::Waypoint wp){return wp.stop_line == 1; }))
  {
    std::cout << "test" << std::endl;

    if(wp.getNextWaypoints().empty())
      continue;
    autoware_map::Waypoint next_wp = *(wp.getNextWaypoints().front());

    lanelet::Point3d pt1 = toLaneletPoint(wp);
    lanelet::Point3d pt2 = toLaneletPoint(next_wp);
    double width = 2;
    lanelet::LineString3d stopline = createStopLineBetweenPoints(pt1, pt2, width, 0.2);

    // create stop sign beside stopline
    lanelet::BasicPoint3d line_bp1 = stopline.front().basicPoint();
    lanelet::BasicPoint3d line_bp2 = stopline.back().basicPoint();

    lanelet::BasicPoint3d sign_bp1 = (6 * line_bp2 - 1 * line_bp1) / 5;
    sign_bp1.z() += 2;
    lanelet::BasicPoint3d sign_bp2 = (4 * line_bp2 - 1 * line_bp1) / 3;
    sign_bp2.z() += 2;

    lanelet::Point3d sign_p1(getId(), sign_bp1);
    lanelet::Point3d sign_p2(getId(), sign_bp2);

    lanelet::LineString3d stop_sign(getId(), {sign_p1, sign_p2});
    stop_sign.attributes()["type"] = "traffic_sign";
    stop_sign.attributes()["subtype"] = "stop_sign";

    auto traffic_sign = lanelet::TrafficSign::make(getId(), lanelet::AttributeMap(), {{stop_sign}, "stop_sign"});
    traffic_sign->addRefLine(stopline);

    lanelet_map_ptr->add(traffic_sign);
  }
}

//keep angles within (M_PI, -M_PI]
double addAngles(double angle1, double angle2)
{
  double sum = angle1 + angle2;
  while( sum > M_PI ) sum -= 2 * M_PI;
  while( sum <= -M_PI ) sum += 2 * M_PI;
  return sum;
}
