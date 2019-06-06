
#include <ros/ros.h>
#include <opendrive2autoware_converter/opendrive_loader.h>
#include <opendrive2autoware_converter/map_writer.h>

void printUsage()
{
  std::cout << "opendrive2autoware_converter is a rosnode that convert OpenDRIVE map format (.xodr) to autoware map format (list of .csv files)." << std::endl << std::endl;
  std::cout << "Commands: " << std::endl;
  std::cout << "        opendrive2autoware_converter _map_file:=Source OpenDRIVE .xodr file name,  _country_codes_dir:=Countries' codes folder name, _save_dir:=Destination folder for Autoware map files, _resolution:=(optional)Waypoints resolution default is 0.5 meters, _keep_right:=(optional) True if car has to drive right lanes default is True" <<std::endl <<std::endl;  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opendrive2autoware_converter");

  std::string src_path;
  std::string dst_path;
  std::string country_codes_path;
  double wp_res = 0.5;
  bool keep_right = true;
  ros::NodeHandle pnh("~");

  if(!pnh.hasParam("map_file")){
    ROS_ERROR_STREAM("You must specify xodr file path!");
    printUsage();
    exit(1);
  }
  if(!pnh.hasParam("country_codes_dir")){
    ROS_ERROR_STREAM("You must specify country code directory path!");
    printUsage();
    exit(1);
  }

  pnh.getParam("map_file", src_path);
  pnh.getParam("country_codes_dir", country_codes_path);
  pnh.param<std::string>("save_dir", dst_path, "./");
  pnh.param<double>("resolution", wp_res, 0.5);
  pnh.param<bool>("keep_right", keep_right, true);
  
  opendrive_converter::OpenDriveLoader map_loader;
  PlannerHNS::RoadNetwork map;
  map_loader.loadOpenDRIVE(src_path, country_codes_path, map, wp_res, keep_right);
  opendrive_converter::MapWriter map_save;
  map_save.writeAutowareMap(dst_path, map);
}
