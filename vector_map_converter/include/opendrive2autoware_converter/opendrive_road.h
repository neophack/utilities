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

#ifndef OPENDRIVE_ROAD
#define OPENDRIVE_ROAD

#include "opendrive_objects.h"

namespace opendrive_converter
{

class FromRoadLink
{
public:
	LINK_TYPE link_type_;
	int from_road_id_;
	CONTACT_POINT contact_point_;

	FromRoadLink(TiXmlElement* main_element)
	{
		if(XmlHelpers::getStringAttribute(main_element, "elementType", "").compare("road") == 0)
			link_type_ = ROAD_LINK;
		else
			link_type_ = JUNCTION_LINK;

		from_road_id_ = XmlHelpers::getIntAttribute(main_element, "elementId", 0);

		if(XmlHelpers::getStringAttribute(main_element, "contactPoint", "").compare("start") == 0)
			contact_point_ = START_POINT;
		else if(XmlHelpers::getStringAttribute(main_element, "contactPoint", "").compare("end") == 0)
			contact_point_ = END_POINT;
		else
			contact_point_ = EMPTY_POINT;
	}
};

class ToRoadLink
{
public:
	LINK_TYPE link_type_;
	int to_road_id_;
	CONTACT_POINT contact_point_;

	ToRoadLink(TiXmlElement* main_element)
	{
		if(XmlHelpers::getStringAttribute(main_element, "elementType", "").compare("road") == 0)
			link_type_ = ROAD_LINK;
		else
			link_type_ = JUNCTION_LINK;

		to_road_id_ = XmlHelpers::getIntAttribute(main_element, "elementId", 0);

		if(XmlHelpers::getStringAttribute(main_element, "contactPoint", "").compare("start") == 0)
			contact_point_ = START_POINT;
		else if(XmlHelpers::getStringAttribute(main_element, "contactPoint", "").compare("end") == 0)
			contact_point_ = END_POINT;
		else
			contact_point_ = EMPTY_POINT;
	}
};

class OpenDriveRoad
{
public:
	std::string name_;
	int id_;
	int junction_id_;
	double length_;
	bool keep_right_;
	std::vector<FromRoadLink> predecessor_road_;
	std::vector<ToRoadLink> successor_road_;
	std::vector<Geometry> geometries_;
	std::vector<Elevation> elevations_;
	std::vector<RoadSection> sections_;
	std::vector<LaneOffset> laneOffsets_;
	std::vector<Signal> road_signals_;
	std::vector<SignalRef> road_signals_references_;
	std::vector<RoadObject> road_objects_;
	std::vector<RoadObjectRef> road_objects_references_;
	std::vector<RoadObjectTunnel> road_objects_tunnels_;
	std::vector<RoadObjectBridge> road_objects_bridges_;
	std::vector<Connection> to_roads_;
	std::vector<Connection> from_roads_;
	std::vector<std::pair<std::string, std::vector<CSV_Reader::LINE_DATA> > >* p_country_signal_codes_;

	std::vector<Connection> getFirstSectionConnections( OpenDriveRoad *_p_predecessor_road);
	std::vector<Connection> getLastSectionConnections( OpenDriveRoad *_p_predecessor_road);
	void getRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list, double resolution = 0.5);
	void getTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights);
	void getTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs);
	void getStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines);
	void insertUniqueToConnection(const Connection& _connection);
	void insertUniqueFromConnection(const Connection& _connection);

	OpenDriveRoad()
	{
		p_country_signal_codes_ = nullptr;
		id_ = 0;
		junction_id_ = 0;
		length_ = 0;
		keep_right_ = true;

	}

	OpenDriveRoad(TiXmlElement* main_element, std::vector<std::pair<std::string, std::vector<CSV_Reader::LINE_DATA> > >* country_signal_codes = nullptr, bool keep_right = true);

	RoadSection* getFirstSection()
	{
		if(sections_.size() == 0)
			return nullptr;

		return &sections_.at(0);
	}

	RoadSection* getLastSection()
	{
		if(sections_.size() == 0)
			return nullptr;

		return &sections_.at(sections_.size()-1);
	}

private:
	Geometry* getMatchingGeometry(const double& sOffset)
	{
		for(unsigned int i=0; i < geometries_.size(); i++)
		{
			if(sOffset >= geometries_.at(i).s && sOffset < (geometries_.at(i).s+geometries_.at(i).length))
			{
				return &geometries_.at(i);
			}
		}

		return nullptr;
	}

	Elevation* getMatchingElevations(const double& sOffset)
	{
		if(elevations_.size() == 0)
			return nullptr;

		if(elevations_.size() == 1)
			return &elevations_.at(0);

		for(int i=1; i < elevations_.size(); i++)
		{
			if(sOffset >= elevations_.at(i-1).s && sOffset < elevations_.at(i).s)
			{
				return &elevations_.at(i-1);
			}
		}

		return &elevations_.at(elevations_.size()-1);
	}

	LaneOffset* getMatchingLaneOffset(const double& sOffset)
	{
		if(laneOffsets_.size() == 0)
			return nullptr;

		if(laneOffsets_.size() == 1)
			return &laneOffsets_.at(0);

		for(int i=1; i < laneOffsets_.size(); i++)
		{
			if(sOffset >= laneOffsets_.at(i-1).s && sOffset < laneOffsets_.at(i).s)
			{
				return &laneOffsets_.at(i-1);
			}
		}

		return &laneOffsets_.at(laneOffsets_.size()-1);
	}

	RoadSection* getMatchingSection(const double& sOffset)
	{
		if(sections_.size() == 0)
			return nullptr;

		if(sections_.size() == 1)
			return &sections_.at(0);

		for(unsigned int i=0; i < sections_.size(); i++)
		{
			double end_offset = sections_.at(i).s_ + sections_.at(i).length_;

			if(sOffset <= end_offset)
			{
				return &sections_.at(i);
			}
		}

		return nullptr;
	}

	RoadSection* getExactMatchingSection(const double& sOffset)
	{
		if(sections_.size() == 0)
			return nullptr;

		if(sections_.size() == 1)
			return &sections_.at(0);

		for(unsigned int i=0; i < sections_.size(); i++)
		{
			if(sOffset == sections_.at(i).s_)
			{
				return &sections_.at(i);
			}
		}

		return nullptr;
	}

	bool createSingleCenterPoint(double _ds, PlannerHNS::WayPoint& _p);

	void insertUniqueFromSectionIds(int from_section_id, const OpenDriveLane* curr_lane, PlannerHNS::Lane& _l);
	void insertUniqueToSectionIds(int to_section_id, const OpenDriveLane* curr_lane, PlannerHNS::Lane& _l);

	void insertUniqueFromRoadIds(int curr_section_id, int curr_lane_id, PlannerHNS::Lane& _l);
	void insertUniqueToRoadIds(int curr_section_id, int curr_lane_id, PlannerHNS::Lane& _l);




	void createRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list);
	void createRoadCenterInfo(std::vector<RoadCenterInfo>& points_list, double resolution = 0.5);
	bool createRoadCenterPoint(RoadCenterInfo& inf_point, double _s);
	void insertRoadCenterInfo(std::vector<RoadCenterInfo>& points_list, RoadCenterInfo& inf_point);
	void fixRedundantPointsLanes(PlannerHNS::Lane& _lane);
	void createSectionPoints(const RoadCenterInfo& ref_info, std::vector<PlannerHNS::Lane>& lanes_list, RoadSection* p_sec, int& wp_id_seq, std::vector<int> &left_lane_ids, std::vector<int> &right_lane_ids);


	PlannerHNS::Lane* getLaneById(const int& _l_id, std::vector<PlannerHNS::Lane>& _lanes_list)
	{
		for(unsigned int i=0; i < _lanes_list.size(); i++)
		{
			if(_lanes_list.at(i).id == _l_id)
				return &_lanes_list.at(i);
		}

		return nullptr;
	}

	bool exists(const std::vector<int>& _list, int _val)
	{
		for(unsigned int j=0; j< _list.size(); j++)
		{
			if(_list.at(j) == _val)
			{
				return true;
			}
		}

		return false;
	}

	OBJECT_TYPE getAutowareMainTypeFromCode(const std::string& country_code, const std::string& type, const std::string& sub_type);
	TRAFFIC_LIGHT_TYPE getAutowareLightTypeFromCode(const std::string& country_code, const std::string& type, const std::string& sub_type);
	ROAD_SIGN_TYPE getAutowareRoadSignTypeFromCode(const std::string& country_code, const std::string& type, const std::string& sub_type);
	ROAD_MARK_TYPE getAutowareRoadMarksTypeFromCode(const std::string& country_code, const std::string& type, const std::string& sub_type);

	OBJECT_TYPE getObjTypeFromText(const std::string& autoware_type);
	TRAFFIC_LIGHT_TYPE getLightTypeFromText(const std::string& autoware_type);
	ROAD_SIGN_TYPE getSignTypeFromText(const std::string& autoware_type);
	ROAD_MARK_TYPE getMarkTypeFromText(const std::string& autoware_type);

};

}

#endif // OPENDRIVE2AUTOWARE_CONVERTER
