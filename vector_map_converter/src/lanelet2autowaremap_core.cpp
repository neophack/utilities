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
#include <vector_map_converter/lanelet2autowaremap.hpp>
#include <amathutils_lib/amathutils.hpp>
#include <geodesy/utm.h>

using namespace lanelet;

#define BLINKER_RIGHT 2
#define BLINKER_LEFT 1
#define SIGNAL_UNKNOWN 0
#define SIGNAL_RED 1
#define SIGNAL_GREEN 2
#define SIGNAL_YELLOW 3


BOOST_GEOMETRY_REGISTER_MULTI_POINT(decltype(std::vector<BasicPoint2d>{} ))

template <class T>
void write(std::ofstream &ofs, std::vector<T> objs)
{
  for( auto obj : objs)
  {
    ofs << obj << std::endl;
  }
}

/**
 * [calculates MGRS x,y from lat/lng information]
 * @method fixPointCoordinate
 * @param  point              [point with lat/lng and information]
 */

void fixPointCoordinate(autoware_map_msgs::Point &point)
{
  geographic_msgs::GeoPoint wgs_point;
  wgs_point.latitude = point.lat;
  wgs_point.longitude = point.lng;
  wgs_point.altitude = point.z;

  geodesy::UTMPoint utm_point;
  geodesy::fromMsg(wgs_point,utm_point);
  point.x = fmod(utm_point.easting,1e5);
  point.y = fmod(utm_point.northing,1e5);
}

void writeAutowareMapMsgs(std::string output_dir,
                          std::vector<autoware_map_msgs::Area> &areas,
                          std::vector<autoware_map_msgs::Lane> &lanes,
                          std::vector<autoware_map_msgs::LaneAttributeRelation> &lane_attribute_relations,
                          std::vector<autoware_map_msgs::LaneChangeRelation> &lane_change_relations,
                          std::vector<autoware_map_msgs::LaneRelation> &lane_relations,
                          std::vector<autoware_map_msgs::LaneSignalLightRelation> &lane_signal_light_relations,
                          std::vector<autoware_map_msgs::OppositeLaneRelation> &opposite_lane_relations,
                          std::vector<autoware_map_msgs::Point> &points,
                          std::vector<autoware_map_msgs::Signal> &signals,
                          std::vector<autoware_map_msgs::SignalLight> &signal_lights,
                          std::vector<autoware_map_msgs::Wayarea> &wayareas,
                          std::vector<autoware_map_msgs::Waypoint> &waypoints,
                          std::vector<autoware_map_msgs::WaypointLaneRelation> &waypoint_lane_relations,
                          std::vector<autoware_map_msgs::WaypointRelation> &waypoint_relations,
                          std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations)
{
  std::ofstream ofs;
  std::string filename;

  //areas
  if(!areas.empty())
  {
    filename = output_dir + "/areas.csv";
    ofs.open(filename);
    ofs << "area_id,point_ids" << std::endl;
    write(ofs, areas);
    ofs.close();
  }
  //lanes
  if(!lanes.empty())
  {
    filename = output_dir + "/lanes.csv";
    ofs.open(filename);
    ofs << "lane_id,start_waypoint_id,end_waypoint_id,lane_number,num_of_lanes,speed_limit,length,width_limit,height_limit,weight_limit" << std::endl;
    write(ofs, lanes);
    ofs.close();
  }
  //lane_attribute_relations
  if(!lane_attribute_relations.empty())
  {
    filename = output_dir + "/lane_attribute_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,attribute_type,area_id" << std::endl;
    write(ofs, lane_attribute_relations);
    ofs.close();
  }
  //lane_change_relations
  if(!lane_change_relations.empty())
  {
    filename = output_dir + "/lane_change_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,next_lane_id,blinker" << std::endl;
    write(ofs, lane_change_relations);
    ofs.close();
  }
  //lane_relations
  if(!lane_relations.empty())
  {
    filename = output_dir + "/lane_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,next_lane_id,blinker" << std::endl;
    write(ofs, lane_relations);
    ofs.close();
  }
  //lane_signal_light_relations
  if(!lane_signal_light_relations.empty())
  {
    filename = output_dir + "/lane_signal_light_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,signal_light_id" << std::endl;
    write(ofs, lane_signal_light_relations);
    ofs.close();
  }
  //opposite_lane_relations
  if(!opposite_lane_relations.empty())
  {
    filename = output_dir + "/opposite_lane_relations.csv";
    ofs.open(filename);
    ofs << "lane_id,opposite_lane_id" << std::endl;
    write(ofs, opposite_lane_relations);
    ofs.close();
  }
  //points
  if(!points.empty())
  {
    filename = output_dir + "/points.csv";
    ofs.open(filename);
    ofs << "point_id,y,x,z,lat,lng,pcd,mgrs,epsg" << std::endl;
    write(ofs, points);
    ofs.close();
  }
  //signals
  if(!signals.empty())
  {
    filename = output_dir + "/signals.csv";
    ofs.open(filename);
    ofs << "signal_id" << std::endl;
    write(ofs, signals);
    ofs.close();
  }
  //signal_lights
  if(!signal_lights.empty())
  {
    filename = output_dir + "/signal_lights.csv";
    ofs.open(filename);
    ofs << "signal_light_id,signal_id,point_id,horizontal_angle,vertical_angle,color_type,arrow_type" << std::endl;
    write(ofs, signal_lights);
    ofs.close();
  }
  //wayareas
  if(!wayareas.empty())
  {
    filename = output_dir + "/wayareas.csv";
    ofs.open(filename);
    ofs << "wayarea_id,area_id" << std::endl;
    write(ofs, wayareas);
    ofs.close();
  }
  //waypoints
  if(!waypoints.empty())
  {
    filename = output_dir + "/waypoints.csv";
    ofs.open(filename);
    ofs << "waypoint_id,point_id,velocity,stop_line,left_width,right_width,height" << std::endl;
    write(ofs, waypoints);
    ofs.close();
  }
  //waypoint_lane_relations
  if(!waypoint_lane_relations.empty())
  {
    filename = output_dir + "/waypoint_lane_relations.csv";
    ofs.open(filename);
    ofs << "waypoint_id,lane_id,order" << std::endl;
    write(ofs, waypoint_lane_relations);
    ofs.close();
  }
  //waypoint_relations
  if(!waypoint_relations.empty())
  {
    filename = output_dir + "/waypoint_relations.csv";
    ofs.open(filename);
    ofs << "waypoint_id,next_waypoint_id,yaw,blinker,distance" << std::endl;
    write(ofs, waypoint_relations);
    ofs.close();
  }
  if(!waypoint_signal_relations.empty())
  {
    filename = output_dir + "/waypoint_signal_relations.csv";
    ofs.open(filename);
    ofs << "waypoint_id,signal_id" << std::endl;
    write(ofs, waypoint_signal_relations);
    ofs.close();
  }
}
autoware_map_msgs::Point convertPoint(const lanelet::ConstPoint3d &lanelet_point, const projection::UtmProjector &projector)
{
  autoware_map_msgs::Point awmap_point;
  GPSPoint utm_point = projector.reverse(lanelet_point.basicPoint());
  awmap_point.point_id = lanelet_point.id();
  awmap_point.x = lanelet_point.x();
  awmap_point.y = lanelet_point.y();
  awmap_point.z = utm_point.ele;
  awmap_point.pcd="";
  awmap_point.mgrs=0;
  awmap_point.epsg = 0;
  awmap_point.lat = utm_point.lat;
  awmap_point.lng = utm_point.lon;
  return awmap_point;
}

autoware_map_msgs::Point convertPoint(const lanelet::Point3d &lanelet_point, const projection::UtmProjector &projector)
{
  lanelet::ConstPoint3d const_point(lanelet_point);
  return convertPoint(const_point, projector);
}

boost::optional<ConstLanelet> getClosestOppositeLane(const LaneletMapPtr map,const routing::RoutingGraphPtr graph, const ConstLanelet &lanelet,const traffic_rules::TrafficRulesPtr traffic_rules)
{
  double padding = 1;
  BasicPoint2d min,max;
  for (auto beside : graph->besides(lanelet))
  {
    for(auto pt : beside.rightBound())
    {
      min.x() = (pt.x() < min.x()) ? pt.x() : min.x();
      min.y() = (pt.y() < min.y()) ? pt.y() : min.y();
      max.x() = (pt.x() > max.x()) ? pt.x() : max.x();
      max.y() = (pt.y() > max.y()) ? pt.y() : max.y();
    }
  }
  min.x() -= padding;
  min.y() -= padding;
  max.x() += padding;
  max.y() += padding;

  auto local_lanelets = map->laneletLayer.search(BoundingBox2d(min,max));

  for (auto beside : graph->besides(lanelet))
  {
    for( auto l : local_lanelets )
    {
      if(!traffic_rules->canPass(l)) continue;
      if ( beside.rightBound() == l.rightBound().invert()) {
        return l;
      }
      if ( beside.leftBound() == l.leftBound().invert()) {
        return l;
      }
    }
  }
  return boost::none;
}

void splitLine(ConstLineString3d laneBound, std::vector<BasicPoint3d> &splitted_points, const double resolution, const int partitions)
{
  splitted_points.push_back(laneBound.front());
  double residue = 0; //residue length from previous segment
  //loop through each segments in bound
  BasicPoint3d prev_pt = laneBound.front();
  for(const BasicPoint3d pt : laneBound)
  {
    if(splitted_points.size() >= partitions) break;
    //continue if no points are made in current segment
    double segment_length = geometry::distance2d(pt, prev_pt);
    if(segment_length + residue < resolution) {
      residue += segment_length;
      prev_pt = pt;
      continue;
    }

    BasicPoint3d direction = ( pt - prev_pt ) / segment_length;
    for(double length = resolution - residue; length < segment_length; length += resolution)
    {
      BasicPoint3d partition_point = prev_pt + direction * length;
      splitted_points.push_back(partition_point);
      if(splitted_points.size() >= partitions) break;
      residue = segment_length - length;
    }
    if(splitted_points.size() >= partitions) break;
    prev_pt = pt;
  }
  splitted_points.push_back(laneBound.back());
}

void set_fine_centerline(Lanelet &lanelet)
{
  double resolution = 1;
  LineString3d centerline;
  std::vector<BasicPoint3d> left_points;
  std::vector<BasicPoint3d> right_points;

  //get length of longer border
  double left_length = geometry::rangedLength(lanelet.leftBound().begin(), lanelet.leftBound().end());
  double right_length = geometry::rangedLength(lanelet.rightBound().begin(), lanelet.rightBound().end());
  double longer_distance = (left_length > right_length) ? left_length : right_length;
  unsigned int partitions = ceil(longer_distance/resolution);
  if(partitions == 0)
  {
    partitions = 1;
  }
  double left_resolution = left_length / partitions;
  double right_resolution = right_length / partitions;

  splitLine(lanelet.leftBound(), left_points, left_resolution, partitions);
  splitLine(lanelet.rightBound(), right_points, right_resolution, partitions);

  if(left_points.size() != partitions + 1 || right_points.size() != partitions + 1) {
    ROS_ERROR_STREAM("something wrong with number of points. (right,left, partitions) " << right_points.size() << ","<< left_points.size() << "," << partitions << "failed to calculate centerline!!!" << std::endl);
    exit(1);
  }
    for(unsigned int i = 0; i < partitions+1; i++)
  {
    BasicPoint3d center_basic_point = (right_points.at(i) + left_points.at(i)) / 2;
    Point3d center_point(utils::getId(), center_basic_point.x(), center_basic_point.y(), center_basic_point.z());
    centerline.push_back(center_point);
  }
  lanelet.setCenterline(centerline);
    return;
}

int getNewId(const int id, const std::unordered_map<int, int> &waypoint_id_correction )
{
  auto new_waypoint_id = waypoint_id_correction.find(id);
  if(new_waypoint_id != waypoint_id_correction.end()) {
    return new_waypoint_id->second;
  }else{
    return id;
  }
}

void createPoints(const std::vector<Lanelet> &vehicle_lanelets, const projection::UtmProjector projector,
                  std::unordered_map<int, autoware_map_msgs::Point> &points_map)
{
  for (const auto &lanelet : vehicle_lanelets)
  {
    for( const auto &center_point : lanelet.centerline())
    {
      autoware_map_msgs::Point awmap_point = convertPoint(center_point, projector);
      points_map[awmap_point.point_id] = awmap_point;
    }
  }
}

void createWaypoints(const std::vector<Lanelet> &vehicle_lanelets, const traffic_rules::TrafficRulesPtr traffic_rules,
                     std::unordered_map<int, autoware_map_msgs::Waypoint> &waypoints_map)
{
  for (const auto &lanelet : vehicle_lanelets)
  {
    for( const auto &center_point : lanelet.centerline())
    {
      //waypoint_id must be same as point_id 
      autoware_map_msgs::Waypoint awmap_waypoint;
      awmap_waypoint.waypoint_id = center_point.id(); 
      awmap_waypoint.point_id = center_point.id();
      awmap_waypoint.velocity = traffic_rules->speedLimit(lanelet).speedLimit.value();
      awmap_waypoint.stop_line = 0;
      double distance_left = geometry::distance2d(center_point, lanelet.leftBound());
      double distance_right= geometry::distance2d(center_point, lanelet.rightBound());
      awmap_waypoint.left_width = distance_left;
      awmap_waypoint.right_width = distance_right;
      awmap_waypoint.height = 0;
      waypoints_map[awmap_waypoint.waypoint_id] = awmap_waypoint;
    }
  }
}

void createWaypointLaneRelations(const std::vector<Lanelet> &vehicle_lanelets,
                                 std::vector<autoware_map_msgs::WaypointLaneRelation> &waypoint_lane_relations)
{
  for (const auto &lanelet : vehicle_lanelets)
  {
    int order = 1;
    for( const auto &center_point : lanelet.centerline() )
    {
      //waypoint lane relation
      autoware_map_msgs::WaypointLaneRelation waypoint_lane_relation;
      waypoint_lane_relation.waypoint_id = center_point.id();
      waypoint_lane_relation.lane_id = lanelet.id();
      waypoint_lane_relation.order = order;
      waypoint_lane_relations.push_back(waypoint_lane_relation);
      order++;
    }
  }
}

void createWaypointRelations(const std::vector<Lanelet> &vehicle_lanelets,
                             std::vector<autoware_map_msgs::WaypointRelation> &waypoint_relations)
{
  for (const auto &lanelet : vehicle_lanelets)
  {
    int order = 1;
    ConstPoint3d prev_center_point;
    for( const auto &center_point : lanelet.centerline() )
    {
      if (order > 1 ) {
        autoware_map_msgs::WaypointRelation waypoint_relation;
        waypoint_relation.waypoint_id = prev_center_point.id();
        waypoint_relation.next_waypoint_id = center_point.id();
        //angle from north(y) in CCW
        waypoint_relation.yaw = M_PI/2 - atan2(center_point.y() - prev_center_point.y(), center_point.x() - prev_center_point.x());
        waypoint_relation.blinker = 0;
        waypoint_relation.distance = geometry::distance2d(center_point, prev_center_point);
        waypoint_relations.push_back(waypoint_relation);
      }
      prev_center_point = center_point;
      order++;
    }
  }
}

void createLanes(const std::vector<Lanelet> &vehicle_lanelets, const traffic_rules::TrafficRulesPtr traffic_rules, const routing::RoutingGraphPtr vehicle_graph,
                 std::unordered_map<int, autoware_map_msgs::Lane> &lanes_map)
{
  for (const auto &lanelet : vehicle_lanelets)
  {
    int order = 1;
    Point3d prev_center_point;
    double width_limit;
    for( const auto &center_point : lanelet.centerline() )
    {
      double distance_left = geometry::distance2d(center_point, lanelet.leftBound());
      double distance_right= geometry::distance2d(center_point, lanelet.rightBound());
      double width = distance_left + distance_right;
      width_limit = (width_limit > width) ? width_limit : width;
    }

    autoware_map_msgs::Lane awmap_lane;
    awmap_lane.lane_id = lanelet.id();
    awmap_lane.start_waypoint_id = lanelet.centerline().front().id();
    awmap_lane.end_waypoint_id = lanelet.centerline().back().id();
    awmap_lane.speed_limit = traffic_rules->speedLimit(lanelet).speedLimit.value();
    awmap_lane.lane_number =vehicle_graph->lefts(lanelet).size() + 1;
    awmap_lane.num_of_lanes = vehicle_graph->besides(lanelet).size() + 1;
    awmap_lane.length = geometry::rangedLength(lanelet.centerline().begin(), lanelet.centerline().end());
    awmap_lane.width_limit = lanelet::geometry::distance(lanelet.rightBound().front(), lanelet.leftBound().front());
    awmap_lane.height_limit = 0;
    awmap_lane.width_limit = width_limit;
    lanes_map[awmap_lane.lane_id] = awmap_lane;
  }
}

void createStopLines(const LineStringLayer &linestring_layer,
                     const std::vector<autoware_map_msgs::WaypointRelation> &waypoint_relations,
                     const std::unordered_map<int, autoware_map_msgs::Point> &points_map,
                     std::unordered_map<int, autoware_map_msgs::Waypoint> &waypoints_map)
{
  //stop_line
  for (const auto &line : linestring_layer)
  {
    if(line.attributeOr(AttributeName::Type, "")==AttributeValueString::StopLine) {
      for(const auto &relation : waypoint_relations)
      {
        
        auto wp = waypoints_map.at(relation.waypoint_id);
                auto next_wp = waypoints_map.at(relation.next_waypoint_id);
                auto next_pt = points_map.at(next_wp.point_id);
                auto pt = points_map.at(wp.point_id);
                double epsilon = 0.1;

        geometry_msgs::Point l1, l2, p1, p2;
        l1.x = line.front().x();
        l1.y = line.front().y();
        l1.z = 0;
        l2.x = line.back().x();
        l2.y = line.back().y();
        l2.z = 0;
        p1.x = pt.x;
        p1.y = pt.y;
        p1.z = 0;
        p2.x = next_pt.x;
        p2.y = next_pt.y;
        p2.z = 0;

                //stopline intersect with current waypoint
        if(amathutils::distanceFromSegment(l1, l2, p1) <= epsilon )
        {
                    waypoints_map.at(relation.waypoint_id).stop_line = 1;
                    continue;
        }
        //stopline intersect with next waypoint
        if(amathutils::distanceFromSegment(l1, l2, p2) <= epsilon) {
                    waypoints_map.at(relation.next_waypoint_id).stop_line = 1;
                    continue;
        }
        //stopline intersets between current and next waypoint
        if (amathutils::isIntersectLine(l1, l2, p1, p2)) {
                    waypoints_map.at(relation.waypoint_id).stop_line = 1;
                  }
      }
    }
  }
}

void createLaneRelations( const std::vector<Lanelet> &vehicle_lanelets,
                          const routing::RoutingGraphPtr vehicle_graph,
                          std::vector<autoware_map_msgs::LaneRelation> &lane_relations)
{
  for (const auto &lanelet : vehicle_lanelets)
  {
    for( const auto &following : vehicle_graph->following(lanelet))
    {
      autoware_map_msgs::LaneRelation lane_relation;
      lane_relation.lane_id = lanelet.id();
      lane_relation.next_lane_id = following.id();
      lane_relation.blinker = 0;
      lane_relations.push_back(lane_relation);
    }
  }
}

void createLaneChangeRelations(const std::vector<Lanelet> &vehicle_lanelets,
                               const routing::RoutingGraphPtr vehicle_graph,
                               std::vector<autoware_map_msgs::LaneChangeRelation> &lane_change_relations)
{
  for (const auto &lanelet : vehicle_lanelets)
  {
    if( vehicle_graph->right(lanelet) ) {
      ConstLanelet right_lanelet = vehicle_graph->right(lanelet).value();
      autoware_map_msgs::LaneChangeRelation lane_change_relation;
      lane_change_relation.lane_id = lanelet.id();
      lane_change_relation.next_lane_id = right_lanelet.id();
      lane_change_relation.blinker = BLINKER_RIGHT;
      lane_change_relations.push_back(lane_change_relation);
    }
    if( vehicle_graph->left(lanelet)) {
      ConstLanelet left_lanelet = vehicle_graph->left(lanelet).value();
      autoware_map_msgs::LaneChangeRelation lane_change_relation;
      lane_change_relation.lane_id = lanelet.id();
      lane_change_relation.next_lane_id = left_lanelet.id();
      lane_change_relation.blinker = BLINKER_LEFT;
      lane_change_relations.push_back(lane_change_relation);
    }
  }
}

void createOppositeLaneRelation(const std::vector<Lanelet> &vehicle_lanelets,
                                const LaneletMapPtr map,
                                const traffic_rules::TrafficRulesPtr traffic_rules,
                                const routing::RoutingGraphPtr vehicle_graph,
                                std::vector<autoware_map_msgs::OppositeLaneRelation> &opposite_lane_relations)
{
  for (const auto &lanelet : vehicle_lanelets)
  {
    auto closest_opposite_opt = getClosestOppositeLane(map,vehicle_graph,lanelet,traffic_rules);
    if(closest_opposite_opt!=boost::none) {
      ConstLanelet closest_opposite = closest_opposite_opt.value();
      for(const auto &opposite_lane : vehicle_graph->besides(closest_opposite))
      {
        autoware_map_msgs::OppositeLaneRelation opposite_lane_relation;
        opposite_lane_relation.lane_id = lanelet.id();
        opposite_lane_relation.opposite_lane_id = opposite_lane.id();
        bool already_added = false;
        for(auto relation : opposite_lane_relations)
        {
          if(relation.lane_id == opposite_lane_relation.lane_id &&
             relation.opposite_lane_id == opposite_lane_relation.opposite_lane_id) {
            already_added =true;
          }
          if(relation.opposite_lane_id == opposite_lane_relation.lane_id &&
             relation.lane_id == opposite_lane_relation.opposite_lane_id) {
            already_added =true;
          }
        }
        if(already_added ==false) {
          opposite_lane_relations.push_back(opposite_lane_relation);
        }
      }
    }
  }
}

void createSignalLights(const std::vector<Lanelet> &vehicle_lanelets,
                        const projection::UtmProjector &projector,
                        std::unordered_map<int, autoware_map_msgs::Point> &points_map,
                        std::vector<autoware_map_msgs::Signal> &signals,
                        std::vector<autoware_map_msgs::SignalLight> &signal_lights)
{
  std::map<Id, bool> checklist;
  for (const auto &lanelet : vehicle_lanelets)
  {
    auto traffic_light_reg_elems = lanelet.regulatoryElementsAs<TrafficLight>();
    for (const auto traffic_light_reg_elem : traffic_light_reg_elems)
    {
      for( const auto traffic_light : traffic_light_reg_elem->trafficLights())
      {

        //skip if it is already added.
        if(checklist.find(traffic_light.id()) != checklist.end() ) continue;
        if(!traffic_light.isLineString())
        {
          ROS_ERROR_STREAM("current converter only supports traffic light expressed in linestrings. (no polygons)");
          continue;
        }

        autoware_map_msgs::Signal awmap_signal;
        awmap_signal.signal_id = traffic_light.id();
        signals.push_back(awmap_signal);
        checklist[traffic_light.id()] = true;

        int color_count = 1;
        ConstLineString3d light_bulbs =  traffic_light.lineString().value();
        for(const auto &bulb : light_bulbs)
        {
          autoware_map_msgs::Point awmap_point;
          awmap_point = convertPoint(ConstPoint3d(bulb), projector);
          points_map[awmap_point.point_id] = awmap_point;

          autoware_map_msgs::SignalLight awmap_signal_light;
          awmap_signal_light.signal_light_id = bulb.id();
          awmap_signal_light.point_id = awmap_point.point_id;
          awmap_signal_light.signal_id = awmap_signal.signal_id;
          //angle from north in CCW
          double horizontal_angle = 0;
          if(lanelet.centerline().size() >= 2)
          {
            int size = lanelet.centerline().size();
            auto p1 = lanelet.centerline()[size - 1];
            auto p2 = lanelet.centerline()[size - 2];
            horizontal_angle = ( M_PI /2 - atan2(p2.y() - p1.y(), p2.x() - p1.x() ) )* 180.0/M_PI;
          }
          awmap_signal_light.horizontal_angle = horizontal_angle;
          awmap_signal_light.vertical_angle = 90; //lamps are always facing parallel to ground
          awmap_signal_light.arrow_type = 0;

          //traffic lights are assumed to be in red, yellow, green order
          if(color_count == 1) awmap_signal_light.color_type = SIGNAL_RED;
          else if (color_count == 2)
            awmap_signal_light.color_type = SIGNAL_YELLOW;
          else if (color_count == 3)
            awmap_signal_light.color_type = SIGNAL_GREEN;
          else awmap_signal_light.color_type = SIGNAL_UNKNOWN;
          color_count++;
          signal_lights.push_back(awmap_signal_light);
        }
      }
    }
  }
}

void createLaneSignalLightRelations( const std::vector<Lanelet> &vehicle_lanelets,
                                     std::vector<autoware_map_msgs::LaneSignalLightRelation> &lane_signal_light_relations)
{
  //signal_light
  for (const auto &lanelet : vehicle_lanelets)
  {
    auto traffic_light_reg_elems = lanelet.regulatoryElementsAs<TrafficLight>();
    for (const auto traffic_light_reg_elem : traffic_light_reg_elems)
    {
      for( const auto traffic_light : traffic_light_reg_elem->trafficLights())
      {
        if(!traffic_light.lineString()) continue;
        ConstLineString3d light_bulbs =  traffic_light.lineString().value();
        for(const auto &bulb : light_bulbs)
        {
          //lane signal relations
          autoware_map_msgs::LaneSignalLightRelation lane_signal_light_relation;
          lane_signal_light_relation.lane_id = lanelet.id();
          lane_signal_light_relation.signal_light_id = bulb.id();
          lane_signal_light_relations.push_back(lane_signal_light_relation);
        }
      }
    }
  }
}

void createWaypointSignalRelations(const std::vector<Lanelet> &vehicle_lanelets,
                                   std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations)
{
  //signal_light
  for (const auto &lanelet : vehicle_lanelets)
  {
    auto traffic_light_reg_elems = lanelet.regulatoryElementsAs<TrafficLight>();
    for (const auto traffic_light_reg_elem : traffic_light_reg_elems)
    {
      ConstLineString3d stopline;
      if(traffic_light_reg_elem->stopLine())
      {
        stopline = traffic_light_reg_elem->stopLine().value();
      }
      else
      {
        ConstPoint3d p1_const = lanelet.leftBound().back();
        ConstPoint3d p2_const = lanelet.rightBound().back();
        Point3d p1(p1_const.id(), p1_const.x(), p1_const.y(), p1_const.z());
        Point3d p2(p2_const.id(), p2_const.x(), p2_const.y(), p2_const.z());
        LineString3d ls(utils::getId(), {p1, p2});
        stopline = ls;
      }

      for( const auto traffic_light : traffic_light_reg_elem->trafficLights())
      {
        if(!traffic_light.lineString()) continue;

        //get closest waypoint from stopline
        int stopping_point_id = 0;
        double min_distance = std::numeric_limits<double>::max();
        for(auto pt : lanelet.centerline())
        {
          double distance = geometry::distance2d(pt, stopline);
          if(distance < min_distance)
          {
            min_distance = distance;
            stopping_point_id = pt.id();
          }
        }
        autoware_map_msgs::WaypointSignalRelation waypoint_signal_relation;
        waypoint_signal_relation.signal_id = traffic_light.lineString()->id();
        waypoint_signal_relation.waypoint_id = stopping_point_id;
        waypoint_signal_relations.push_back(waypoint_signal_relation);
      }
    }
  }
}

void getAllConflictingLanelets( const Lanelet &lanelet,
                                const routing::RoutingGraphPtr vehicle_graph,
                                std::vector<ConstLanelet> &conflicting_lanelets,
                                std::set<Id> &already_calculated)
{
  const auto conflicting_lanelets_or_areas = vehicle_graph->conflicting(lanelet);
  if(conflicting_lanelets_or_areas.size() == 0) return;
  conflicting_lanelets.push_back(lanelet);
  already_calculated.insert(lanelet.id());
  for ( auto conflicting_lanelet_area : conflicting_lanelets_or_areas)
  {
    if(!conflicting_lanelet_area.lanelet()) continue;
    ConstLanelet conflicting_lanelet = conflicting_lanelet_area.lanelet().value();
    if(already_calculated.find(conflicting_lanelet.id()) != already_calculated.end()) continue;
    conflicting_lanelets.push_back(conflicting_lanelet);
    already_calculated.insert(conflicting_lanelet.id());
  }

  //continue if it is only conflicting with areas
  if(conflicting_lanelets.size() == 1) {
    return;
  }
  //get all linked intersecting lanes
  std::stack<ConstLanelet> stack;
  //initial setup for stack
  for ( auto conflicting_lanelet : conflicting_lanelets)
  {
    stack.push(conflicting_lanelet);
  }
  while(!stack.empty()) {
    const auto tmp_conflicting_lanelets_or_areas = vehicle_graph->conflicting(stack.top());
    stack.pop();

    for (const auto &la : tmp_conflicting_lanelets_or_areas)
    {
      if(!la.lanelet()) continue;
      ConstLanelet l = la.lanelet().value();
      if(already_calculated.find(l.id()) == already_calculated.end()) {
        stack.push(l);
        conflicting_lanelets.push_back(l);
        already_calculated.insert(l.id());
      }
    }
  }
  return;
}

void createIntersections(const std::vector<Lanelet> &vehicle_lanelets,
                         const routing::RoutingGraphPtr vehicle_graph,
                         const projection::UtmProjector &projector,
                         std::unordered_map<int, autoware_map_msgs::Point> &points_map,
                         std::vector<autoware_map_msgs::Area> &areas,
                         std::vector<autoware_map_msgs::LaneAttributeRelation> &lane_attribute_relations)
{
  std::set<Id> already_calculated;

  for (const auto &lanelet : vehicle_lanelets)
  {
    //skip if lanelet is already used
    if(already_calculated.find(lanelet.id()) != already_calculated.end()) {
      continue;
    }
    std::vector<ConstLanelet> conflicting_lanelets;
    getAllConflictingLanelets(lanelet, vehicle_graph, conflicting_lanelets, already_calculated);    
    if(conflicting_lanelets.size() <= 1) continue;
    
    //calculate convex hull
    std::vector<BasicPoint2d> vertices;
    std::vector<BasicPoint2d> hull;
    for ( const auto &conflicting_lanelet : conflicting_lanelets)
    {
      for( const auto &pt : conflicting_lanelet.polygon3d())
      {
        vertices.push_back(utils::to2D(pt.basicPoint()));
      }
    }
    boost::geometry::convex_hull(vertices, hull);
    std::vector<int> ids;
    for( const auto &pt : hull)
    {
      autoware_map_msgs::Point awmap_point;
      const auto basic_pt3d = utils::to3D(pt);
      Point3d pt3d(utils::getId(), basic_pt3d.x(), basic_pt3d.y(), basic_pt3d.z());
      awmap_point = convertPoint(ConstPoint3d(pt3d), projector);
      points_map[awmap_point.point_id] = awmap_point;
      ids.push_back(awmap_point.point_id);
    }
    //intersection area
    autoware_map_msgs::Area awmap_area;
    awmap_area.area_id = utils::getId();
    awmap_area.point_ids = ids;
    areas.push_back(awmap_area);

    //lane attribute relations
    autoware_map_msgs::LaneAttributeRelation lane_attribute_relation;
    lane_attribute_relation.attribute_type = autoware_map_msgs::LaneAttributeRelation::INTERSECTION;
    lane_attribute_relation.area_id = awmap_area.area_id;
    for ( const auto &conflicting_lanelet : conflicting_lanelets)
    {
      lane_attribute_relation.lane_id = conflicting_lanelet.id();
      lane_attribute_relations.push_back(lane_attribute_relation);
    }
  }
}

void createCrossWalks(const std::vector<Lanelet> &vehicle_lanelets,
                      const routing::RoutingGraphContainer &overall_graphs,
                      const projection::UtmProjector &projector,
                      std::unordered_map<int, autoware_map_msgs::Point> &points_map,
                      std::vector<autoware_map_msgs::Area> &areas,
                      std::vector<autoware_map_msgs::LaneAttributeRelation> &lane_attribute_relations )
{
  std::set<Id> already_calculated;
  for (const auto &lanelet : vehicle_lanelets)
  {
    const auto cross_walk_lanelets = overall_graphs.conflictingInGraph(lanelet,1);
    for ( const auto &cross_walk_lanelet : cross_walk_lanelets)
    {
      int cross_walk_area_id;
      if(already_calculated.find(cross_walk_lanelet.id()) == already_calculated.end() )
      {
        std::vector<ConstPoint3d> vertices;
        for( const auto &pt : cross_walk_lanelet.polygon3d())
        {
          Point3d point3d(pt);
          point3d.setId(utils::getId());
          vertices.push_back(pt);
        }

        //add points
        std::vector<int> ids;
        for( const auto &pt : vertices)
        {
          autoware_map_msgs::Point awmap_point;
          awmap_point = convertPoint(pt, projector);
          points_map[awmap_point.point_id] = awmap_point;
          ids.push_back(awmap_point.point_id);
        }

        //cross walk area
        autoware_map_msgs::Area awmap_area;
        awmap_area.area_id = cross_walk_lanelet.id();
        awmap_area.point_ids = ids;
        areas.push_back(awmap_area);

        already_calculated.insert(cross_walk_lanelet.id());
      }

      autoware_map_msgs::LaneAttributeRelation lane_attribute_relation;
      lane_attribute_relation.lane_id = lanelet.id();
      lane_attribute_relation.attribute_type = autoware_map_msgs::LaneAttributeRelation::CROSS_WALK;
      lane_attribute_relation.area_id = cross_walk_lanelet.id();
      lane_attribute_relations.push_back(lane_attribute_relation);
    }
  }
}

void removeOverlappingWaypoints(  const std::unordered_map<int, autoware_map_msgs::Point> points_map,
                                  std::unordered_map<int, autoware_map_msgs::Waypoint> &waypoints_map,
                                  std::vector<autoware_map_msgs::Lane> &lanes,
                                  std::vector<autoware_map_msgs::WaypointLaneRelation> &waypoint_lane_relations,
                                  std::vector<autoware_map_msgs::WaypointRelation> &waypoint_relations,
                                  std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations )
{
  //preparation to remove overlapping waypoint
  std::set<Id> wp_checklist;
  std::unordered_map<int, int> waypoint_id_correction;

  std::vector<int> start_end_points;
  for(const auto &lane : lanes)
  {
    start_end_points.push_back(lane.start_waypoint_id);
    start_end_points.push_back(lane.end_waypoint_id);
  }

  const double epsilon = 1e-6;
  for (int i = 0; i < start_end_points.size(); i++)
  {
    // for (auto i_id : start_end_points)
    Id i_id = start_end_points.at(i);
    if(wp_checklist.find(i_id) != wp_checklist.end()) continue;
    std::vector<Id> same_points;
    same_points.push_back(i_id);
    for(int j = i+1; j < start_end_points.size(); j++)
    {
      // for(auto j_id : start_end_points)
      Id j_id = start_end_points.at(j);
      if (i_id == j_id || wp_checklist.find(i_id) != wp_checklist.end() ) continue;
      //find position of waypoint using waypoint_id = point_id
      auto i_wp = waypoints_map.at(i_id);
      auto j_wp = waypoints_map.at(j_id);
      auto i_point = points_map.at(i_wp.point_id);
      auto j_point = points_map.at(j_wp.point_id);
      auto i_point3d = Point3d(i_point.x, i_point.y, i_point.z);
      auto j_point3d = Point3d(j_point.x, j_point.y, j_point.z);
      if(geometry::distance2d(i_point3d, j_point3d) < epsilon) {
        same_points.push_back(j_id);
      }
    }
    Id smallest_id = i_id;
    for(const auto id : same_points)
    {
      if(smallest_id > id) smallest_id = id;
    }
    for(const auto id : same_points)
    {
      wp_checklist.insert(id);
      if(id == smallest_id) continue;
      waypoint_id_correction[id] = smallest_id;
    }
  }

  //rewrite waypoint_id in relations 
  for(auto &lane : lanes)
  {
    lane.start_waypoint_id = getNewId(lane.start_waypoint_id, waypoint_id_correction);
    lane.end_waypoint_id = getNewId(lane.end_waypoint_id, waypoint_id_correction);
  }
  for(auto &relation : waypoint_lane_relations)
  {
    relation.waypoint_id = getNewId(relation.waypoint_id, waypoint_id_correction);
  }
  for(auto &relation : waypoint_relations)
  {
    relation.waypoint_id = getNewId(relation.waypoint_id, waypoint_id_correction);
    relation.next_waypoint_id = getNewId(relation.next_waypoint_id, waypoint_id_correction);
  }
  for(auto &relation : waypoint_signal_relations)
  {
    relation.waypoint_id = getNewId(relation.waypoint_id, waypoint_id_correction);
  }
  
  //remove overlapping waypoints
  for(auto itr =  waypoints_map.begin(); itr != waypoints_map.end();)
  {
    if (waypoint_id_correction.find(itr->first) != waypoint_id_correction.end())
    {
      itr = waypoints_map.erase(itr);
    }
    else
    {
      ++itr;
    }
  }
}

void convertLanelet2AutowareMap(LaneletMapPtr map,
                                projection::UtmProjector projector,
                                std::vector<autoware_map_msgs::Area> &areas,
                                std::vector<autoware_map_msgs::Lane> &lanes,
                                std::vector<autoware_map_msgs::LaneAttributeRelation> &lane_attribute_relations,
                                std::vector<autoware_map_msgs::LaneChangeRelation> &lane_change_relations,
                                std::vector<autoware_map_msgs::LaneRelation> &lane_relations,
                                std::vector<autoware_map_msgs::LaneSignalLightRelation> &lane_signal_light_relations,
                                std::vector<autoware_map_msgs::OppositeLaneRelation> &opposite_lane_relations,
                                std::vector<autoware_map_msgs::Point> &points,
                                std::vector<autoware_map_msgs::Signal> &signals,
                                std::vector<autoware_map_msgs::SignalLight> &signal_lights,
                                std::vector<autoware_map_msgs::Wayarea> &wayareas,
                                std::vector<autoware_map_msgs::Waypoint> &waypoints,
                                std::vector<autoware_map_msgs::WaypointLaneRelation> &waypoint_lane_relations,
                                std::vector<autoware_map_msgs::WaypointRelation> &waypoint_relations,
                                std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations)
{



  traffic_rules::TrafficRulesPtr traffic_rules =
    traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  routing::RoutingGraphPtr vehicle_graph = routing::RoutingGraph::build(*map, *traffic_rules);
  traffic_rules::TrafficRulesPtr pedestrian_rules =
    traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian);
  routing::RoutingGraphConstPtr pedestrian_graph = routing::RoutingGraph::build(*map, *pedestrian_rules);
  routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});

  //get Lanes
  int waypoint_id = 1;
  int point_id = 1;
  int lane_id = 1;
  std::unordered_map<Id, int> lane_id_table;
  std::unordered_map<int, autoware_map_msgs::Lane> lanes_map;
  std::unordered_map<int, autoware_map_msgs::Point> points_map;
  std::unordered_map<int, autoware_map_msgs::Waypoint> waypoints_map;



  std::vector<Lanelet> vehicle_lanelets;

  for (const auto &lanelet : map->laneletLayer)
  {
    if(traffic_rules->canPass(lanelet)) {
      if(lanelet.leftBound().empty() || lanelet.rightBound().empty()) {
        continue;
      }
      vehicle_lanelets.push_back(lanelet);
      lane_id_table[lanelet.id()] = lane_id++;
    }
  }

  for( auto & lanelet : vehicle_lanelets)
  {
    set_fine_centerline(lanelet);
  }

    createPoints(vehicle_lanelets, projector, points_map);
    createWaypoints(vehicle_lanelets, traffic_rules, waypoints_map);
    createLanes(vehicle_lanelets, traffic_rules, vehicle_graph, lanes_map);
    createLaneRelations(vehicle_lanelets, vehicle_graph, lane_relations);
    createLaneChangeRelations(vehicle_lanelets, vehicle_graph, lane_change_relations);
    createOppositeLaneRelation(vehicle_lanelets, map, traffic_rules, vehicle_graph, opposite_lane_relations);
    createWaypointLaneRelations(vehicle_lanelets, waypoint_lane_relations);
    createSignalLights(vehicle_lanelets, projector, points_map,signals, signal_lights);
    createWaypointRelations(vehicle_lanelets, waypoint_relations);
    createLaneSignalLightRelations(vehicle_lanelets, lane_signal_light_relations);
    createWaypointSignalRelations(vehicle_lanelets, waypoint_signal_relations);
    createStopLines(map->lineStringLayer, waypoint_relations, points_map, waypoints_map);
    createIntersections(vehicle_lanelets, vehicle_graph, projector, points_map, areas, lane_attribute_relations);
    createCrossWalks(vehicle_lanelets, overall_graphs, projector, points_map, areas, lane_attribute_relations);
  
  removeOverlappingWaypoints(points_map, waypoints_map, lanes, waypoint_lane_relations, waypoint_relations, waypoint_signal_relations);

  for(const auto &item : waypoints_map)
  {
    waypoints.push_back(item.second);
  }
  for(const auto &item : lanes_map)
  {
    lanes.push_back(item.second);
  }
  for(const auto &item : points_map)
  {
    points.push_back(item.second);
  }
  
  for( auto &point : points)
  {
    fixPointCoordinate(point);
  }  
}
