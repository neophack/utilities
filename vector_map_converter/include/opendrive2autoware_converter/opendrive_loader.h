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

#ifndef OPENDRIVE2AUTOWARE_CONVERTER
#define OPENDRIVE2AUTOWARE_CONVERTER

#include "opendrive_road.h"
#include "dirent.h"

namespace opendrive_converter
{
class OpenDriveLoader
{

public:
	OpenDriveLoader();
    ~OpenDriveLoader();
    void getFileNameInFolder(const std::string& path, std::vector<std::string>& out_list);
    void loadCountryCods(const std::string& codes_csv_folder);
    void loadOpenDRIVE(const std::string& xodr_file, const std::string& codes_folder, PlannerHNS::RoadNetwork& map,double resolution = 0.5, bool keep_right = true);
    void getMapLanes(std::vector<PlannerHNS::Lane>& all_lanes, double resolution = 0.5);
    void getTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights);
    void getTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs);
    void getStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines);
    void connectRoads();

private:
		bool keep_right_;
    std::vector<OpenDriveRoad> roads_list_;
    std::vector<Junction> junctions_list_;
    std::vector<std::pair<std::string, std::vector<CSV_Reader::LINE_DATA> > > country_signal_codes_;


    std::vector<OpenDriveRoad*> getRoadsBySuccId(int _id);
    std::vector<OpenDriveRoad*> getRoadsByPredId(int _id);

    OpenDriveRoad* getRoadById(int _id);
    Junction* getJunctionById(int _id);
    void linkWayPoints(PlannerHNS::RoadNetwork& map);
    void linkLanesPointers(PlannerHNS::RoadNetwork& map);
};

}

#endif // OPENDRIVE2AUTOWARE_CONVERTER
