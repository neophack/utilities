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

#ifndef OPENDRIVE2OP_CONVERTER
#define OPENDRIVE2OP_CONVERTER

#include "opendrive2autoware_converter/opendrive_road.h"

namespace opendrive_converter
{
class MapWriter
{

public:
	MapWriter();
    ~MapWriter();
    void writeAutowareMap(std::string folder_name, PlannerHNS::RoadNetwork& map);


private:

	template <class T>
	void writeCSVFile(const std::string& folder, const std::string& title, const std::string& header, const std::vector<T>& data_list);
	PlannerHNS::Lane* getLaneFromID(PlannerHNS::RoadNetwork& map, int _l_id)
	{
		for(unsigned int i=0; i<map.roadSegments.size(); i++)
		{
			for(unsigned int j=0; j<map.roadSegments.at(i).Lanes.size(); j++)
			{
				if(map.roadSegments.at(i).Lanes.at(j).points.size() > 0 &&  map.roadSegments.at(i).Lanes.at(j).id == _l_id)
				{
					return &map.roadSegments.at(i).Lanes.at(j);
				}
			}
		}

		return nullptr;
	}
};

}

#endif // OPENDRIVE2AUTOWARE_CONVERTER
