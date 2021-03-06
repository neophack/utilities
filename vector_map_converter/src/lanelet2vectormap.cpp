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
#include <vector_map_converter/lanelet2vectormap.hpp>
#include <vector_map_converter/autoware2vectormap.hpp>
#include <vector_map_converter/lanelet2autowaremap.hpp>

void printUsage()
{
  std::cout << "Required parameters" << std::endl;
  std::cout << "_map_file:=<path to laneletmap>" << std::endl;
  std::cout << "_origin_lat:=latitude origin used for xyz projection>" << std::endl;
  std::cout << "_origin_lon:=<longitude origin used for xyz projection>" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lanelet2vectormap_converter");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    if(!pnh.hasParam("map_file")){
      ROS_ERROR_STREAM("You must specify file path!");
      printUsage();
      exit(1);
    }
    if(!pnh.hasParam("origin_lat") || !pnh.hasParam("origin_lon") ){
      ROS_ERROR_STREAM("You must specify an origin!");
      printUsage();
      exit(1);
    }

    std::string map_file_path;
    std::string save_dir;
    double origin_lat;
    double origin_lon;
    bool wayareas_from_lanes;
    pnh.param<std::string>("map_file", map_file_path, "");
    pnh.param<std::string>("save_dir", save_dir, "./");
    pnh.param<double>("origin_lat", origin_lat, 0.0);
    pnh.param<double>("origin_lon", origin_lon,0.0);
    pnh.param<bool>("wayareas_from_lanes", wayareas_from_lanes, false);


    std::vector<autoware_map_msgs::Point> points;
    std::vector<autoware_map_msgs::Lane> lanes;
    std::vector<autoware_map_msgs::LaneAttributeRelation> lane_attribute_relations;
    std::vector<autoware_map_msgs::LaneRelation> lane_relations;
    std::vector<autoware_map_msgs::LaneSignalLightRelation> lane_signal_light_relations;
    std::vector<autoware_map_msgs::LaneChangeRelation> lane_change_relations;
    std::vector<autoware_map_msgs::OppositeLaneRelation> opposite_lane_relations;
    std::vector<autoware_map_msgs::Area> areas;
    std::vector<autoware_map_msgs::Signal> signals;
    std::vector<autoware_map_msgs::SignalLight> signal_lights;
    std::vector<autoware_map_msgs::Wayarea> wayareas;
    std::vector<autoware_map_msgs::Waypoint> waypoints;
    std::vector<autoware_map_msgs::WaypointLaneRelation> waypoint_lane_relations;
    std::vector<autoware_map_msgs::WaypointRelation> waypoint_relations;
    std::vector<autoware_map_msgs::WaypointSignalRelation> waypoint_signal_relations;

    std::vector<vector_map_msgs::Area> vmap_areas;
    std::vector<vector_map_msgs::CrossRoad> vmap_cross_roads;
    std::vector<vector_map_msgs::CrossWalk> vmap_cross_walks;
    std::vector<vector_map_msgs::DTLane> vmap_dtlanes;
    std::vector<vector_map_msgs::Lane> vmap_lanes;
    std::vector<vector_map_msgs::Line> vmap_lines;
    std::vector<vector_map_msgs::Node> vmap_nodes;
    std::vector<vector_map_msgs::Point> vmap_points;
    std::vector<vector_map_msgs::Pole> vmap_dummy_poles;
    std::vector<vector_map_msgs::RoadSign> vmap_road_signs;
    std::vector<vector_map_msgs::Signal> vmap_signals;
    std::vector<vector_map_msgs::StopLine> vmap_stop_lines;
    std::vector<vector_map_msgs::UtilityPole> vmap_dummy_uility_poles;
    std::vector<vector_map_msgs::Vector> vmap_vectors;
    std::vector<vector_map_msgs::WayArea> vmap_way_areas;
    std::vector<vector_map_msgs::WhiteLine> vmap_white_lines;

    using namespace lanelet;
    ErrorMessages errors;
    projection::UtmProjector projector(Origin({origin_lat,origin_lon}));
    std::cerr << map_file_path << std::endl;
    LaneletMapPtr map = load(map_file_path, projector, &errors);
    autoware_map::AutowareMapHandler map_handler;

    // using namespace lanelet;
    // ErrorMessages errors;
    // projection::UtmProjector projector(Origin({40.68608001822,-89.59589859017}));
    // LaneletMapPtr map = load("/home/mitsudome-r/data/peoria/lanelet_map/Peoria_2_24_2019_fixed.osm", projector,&errors);
    //
    // autoware_map::AutowareMapHandler map_handler;

    convertLanelet2AutowareMap(map,
                               projector,
                               areas,
                               lanes,
                               lane_attribute_relations,
                               lane_change_relations,
                               lane_relations,
                               lane_signal_light_relations,
                               opposite_lane_relations,
                               points,
                               signals,
                               signal_lights,
                               wayareas,
                               waypoints,
                               waypoint_lane_relations,
                               waypoint_relations,
                               waypoint_signal_relations);

    map_handler.setFromAutowareMapMsgs(areas,
                                       lanes,
                                       lane_attribute_relations,
                                       lane_change_relations,
                                       lane_relations,
                                       lane_signal_light_relations,
                                       opposite_lane_relations,
                                       points,
                                       signals,
                                       signal_lights,
                                       wayareas,
                                       waypoints,
                                       waypoint_lane_relations,
                                       waypoint_relations,
                                       waypoint_signal_relations
                                       );
    map_handler.resolveRelations();

    getVectorMapMsgs( map_handler,
                      vmap_areas,
                      vmap_cross_roads,
                      vmap_cross_walks,
                      vmap_dtlanes,
                      vmap_lanes,
                      vmap_lines,
                      vmap_nodes,
                      vmap_points,
                      vmap_dummy_poles,
                      vmap_road_signs,
                      vmap_signals,
                      vmap_stop_lines,
                      vmap_dummy_uility_poles,
                      vmap_vectors,
                      vmap_way_areas,
                      vmap_white_lines
                      );

    complementVectorMap(map,
                        projector,
                        vmap_lines,
                        vmap_points,
                        vmap_dummy_poles,
                        vmap_road_signs,
                        vmap_stop_lines,
                        vmap_vectors,
                        vmap_white_lines
                        );

    if( wayareas_from_lanes )
    {
       createWayAreasFromLanes(map_handler, vmap_way_areas, vmap_areas, vmap_lines, vmap_points);
    }

    writeVectorMapMsgs( save_dir,
                        vmap_areas,
                        vmap_cross_roads,
                        vmap_cross_walks,
                        vmap_dtlanes,
                        vmap_lanes,
                        vmap_lines,
                        vmap_nodes,
                        vmap_points,
                        vmap_dummy_poles,
                        vmap_road_signs,
                        vmap_signals,
                        vmap_stop_lines,
                        vmap_dummy_uility_poles,
                        vmap_vectors,
                        vmap_way_areas,
                        vmap_white_lines
                        );
}
