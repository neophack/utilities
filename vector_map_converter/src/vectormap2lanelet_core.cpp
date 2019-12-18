
#include <vector_map_converter/vectormap2lanelet.hpp>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

using VectorMap = vector_map::VectorMap;
using Node2Angle = std::unordered_map<int, double>;
using LaneKey = vector_map::Key<vector_map::Lane>;
using NodeKey = vector_map::Key<vector_map::Node>;
using PointKey = vector_map::Key<vector_map::Point>;
using DTLaneKey = vector_map::Key<vector_map::DTLane>;
using AreaKey = vector_map::Key<vector_map::Area>;
using LineKey = vector_map::Key<vector_map::Line>;
using SignalKey = vector_map::Key<vector_map::Signal>;
using VectorKey = vector_map::Key<vector_map::Vector>;
using StopLineKey = vector_map::Key<vector_map::StopLine>;
// using LaneKey = vector_map::Key<vector_map::Lane>;

using lanelet::utils::getId;
using V2L = std::map<int, lanelet::Id>;
using L2V = std::map<lanelet::Id, int>;

bool exists(std::unordered_set<lanelet::Id> set, lanelet::Id id)
{
  return set.find(id) != set.end();
}

void convertVectorMap2Lanelet2(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap)
{
  V2L v2l_id;
  L2V l2v_id;
  const auto node2angle = getNode2Angle(vmap);
  convertLanes(vmap, lmap, node2angle, &v2l_id, &l2v_id);
  addIntersectionTags(vmap, lmap);
  addCrossWalks(vmap, lmap);
  connectLanelets(vmap, lmap, v2l_id, l2v_id);
  addTrafficLights(vmap,lmap);
  transformPoint(lmap);
  // SimplifyLineString(lmap);
}

lanelet::Point3d movePointByDirection(lanelet::Point3d p, double angle, double distance)
{
  double dir_x = std::cos(angle);
  double dir_y = std::sin(angle);

  dir_x *= distance;
  dir_y *= distance;

  double x = p.x() + dir_x;
  double y = p.y() + dir_y;

  lanelet::Point3d moved_point(lanelet::utils::getId(), x, y, p.z());
  return moved_point;
}

lanelet::Lanelet createLanelet(const VectorMap& vmap, const int lnid, const Node2Angle& node2angle)
{
  const auto& ln = vmap.findByKey(LaneKey(lnid));
  const auto& bn = vmap.findByKey(NodeKey(ln.bnid));
  const auto& bn_point = vmap.findByKey(PointKey(bn.pid));
  const auto& fn = vmap.findByKey(NodeKey(ln.fnid));
  const auto& fn_point = vmap.findByKey(PointKey(fn.pid));
  const auto& dtlane = vmap.findByKey(DTLaneKey(ln.did));
  double left_width = 1.5;
  double right_width = 1.5;

  if (dtlane.did != 0)
  {
    left_width = dtlane.lw;
    right_width = dtlane.rw;
  }

  auto lanelet_point_bn = lanelet::Point3d(lanelet::utils::getId(), bn_point.bx, bn_point.ly, bn_point.h);
  auto lanelet_point_fn = lanelet::Point3d(lanelet::utils::getId(), fn_point.bx, fn_point.ly, fn_point.h);

  double angle = getLaneAngle(vmap, lnid);

  auto bn_angle = node2angle.at(bn.nid);
  auto fn_angle = node2angle.at(fn.nid);

  auto left_p1 = movePointByDirection(lanelet_point_bn, bn_angle - M_PI / 2, left_width);
  auto left_p2 = movePointByDirection(lanelet_point_fn, fn_angle - M_PI / 2, left_width);
  auto right_p1 = movePointByDirection(lanelet_point_bn, bn_angle + M_PI / 2, right_width);
  auto right_p2 = movePointByDirection(lanelet_point_fn, fn_angle + M_PI / 2, right_width);

  lanelet::LineString3d left_bound(getId(), { left_p1, left_p2 });
  lanelet::LineString3d right_bound(getId(), { right_p1, right_p2 });

  lanelet::Lanelet lanelet(getId(), left_bound, right_bound);
  return lanelet;
}

double getLaneAngle(const VectorMap& vmap, int lnid)
{
  const auto& ln = vmap.findByKey(LaneKey(lnid));
  const auto& bn = vmap.findByKey(NodeKey(ln.bnid));
  const auto& bn_point = vmap.findByKey(PointKey(bn.pid));
  const auto& fn = vmap.findByKey(NodeKey(ln.fnid));
  const auto& fn_point = vmap.findByKey(PointKey(fn.pid));

  return std::atan2(fn_point.ly - bn_point.ly, fn_point.bx - bn_point.bx);
}

Node2Angle getNode2Angle(const VectorMap& vmap)
{
  Node2Angle node2angle;
  for (auto vmap_node : vmap.findByFilter([](vector_map::Node n) { return true; }))
  {
    auto b_lanes = vmap.findByFilter([&](vector_map::Lane l) { return l.bnid == vmap_node.nid; });
    auto f_lanes = vmap.findByFilter([&](vector_map::Lane l) { return l.fnid == vmap_node.nid; });

    std::vector<double> angles;
    for (const auto& l : b_lanes)
    {
      angles.push_back(getLaneAngle(vmap, l.lnid));
    }
    for (const auto& l : f_lanes)
    {
      angles.push_back(getLaneAngle(vmap, l.lnid));
    }
    node2angle[vmap_node.nid] = getAngleAverage(angles);
  }
  return node2angle;
}

void convertLanes(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap, const Node2Angle& node2angle, V2L* v2l_id,
                  L2V* l2v_id)
{
  for (auto vmap_lane : vmap.findByFilter([](vector_map::Lane l) { return true; }))
  {
    const auto& lanelet = createLanelet(vmap, vmap_lane.lnid, node2angle);
    lmap->add(lanelet);
    (*v2l_id)[vmap_lane.lnid] = lanelet.id();
    (*l2v_id)[lanelet.id()] = vmap_lane.lnid;
  }
}

double getAngleAverage(const std::vector<double>& angles)
{
  double sum_x, sum_y, avg_x, avg_y;
  sum_x = sum_y = avg_x = avg_y = 0;

  for (double angle : angles)
  {
    sum_x += std::cos(angle);
    sum_y += std::sin(angle);
  }

  avg_x = sum_x / angles.size();
  avg_y = sum_y / angles.size();

  return std::atan2(avg_y, avg_x);
}

lanelet::Polygon3d convertToPolygon(const VectorMap& vmap, int area_id)
{
  auto area = vmap.findByKey(AreaKey(area_id));
  auto line = vmap.findByKey(LineKey(area.slid));

  auto bp = vmap.findByKey(PointKey(line.bpid));
  lanelet::Point3d lanelet_bp = lanelet::Point3d(getId(), bp.bx, bp.ly, bp.h);
  std::vector<lanelet::Point3d> polygon_points;
  polygon_points.push_back(lanelet_bp);
  while (line.lid != 0)
  {
    auto fp = vmap.findByKey(PointKey(line.fpid));
    lanelet::Point3d lanelet_fp = lanelet::Point3d(getId(), fp.bx, fp.ly, fp.h);
    polygon_points.push_back(lanelet_fp);
    line = vmap.findByKey(LineKey(line.flid));
  }
  lanelet::Polygon3d polygon(getId(), polygon_points);
  return polygon;
}

double getLineLength(const VectorMap& vmap, int line_id)
{
  auto line = vmap.findByKey(LineKey(line_id));
  auto bp = vmap.findByKey(PointKey(line.bpid));
  auto fp = vmap.findByKey(PointKey(line.fpid));
  return std::hypot(fp.ly - bp.ly, fp.bx - bp.bx);
}

double getLineAngle(const VectorMap& vmap, int line_id)
{
  auto line = vmap.findByKey(LineKey(line_id));
  auto bp = vmap.findByKey(PointKey(line.bpid));
  auto fp = vmap.findByKey(PointKey(line.fpid));
  return std::atan2(fp.ly - bp.ly, fp.bx - bp.bx);
}

lanelet::LineString3d convertToLineString(const VectorMap& vmap, int line_id)
{
  auto line = vmap.findByKey(LineKey(line_id));
  auto bp = vmap.findByKey(PointKey(line.bpid));
  auto fp = vmap.findByKey(PointKey(line.fpid));

  lanelet::Point3d p1(getId(), bp.bx, bp.ly, bp.h);
  lanelet::Point3d p2(getId(), fp.bx, fp.ly, fp.h);

  return lanelet::LineString3d(getId(), { p1, p2 });
}

lanelet::LineString3d convertToLineString(const VectorMap& vmap, std::pair<int, int> straight_line,
                                          std::unordered_set<long int>& done)
{
  auto line = vmap.findByKey(LineKey(straight_line.first));
  std::vector<lanelet::Point3d> points;
  auto bp = vmap.findByKey(PointKey(line.bpid));
  lanelet::Point3d p1(getId(), bp.bx, bp.ly, bp.h);
  points.push_back(p1);

  while (line.lid != 0 && line.blid != straight_line.second)
  {
    done.insert(line.lid);
    auto fp = vmap.findByKey(PointKey(line.fpid));
    lanelet::Point3d p2(getId(), fp.bx, fp.ly, fp.h);
    points.push_back(p2);
    line = vmap.findByKey(LineKey(line.flid));
  }
  // if(line.lid != 0)
  // {
  //   auto fp = vmap.findByKey(PointKey(line.fpid));
  //   lanelet::Point3d p3(getId(), fp.bx, fp.ly, fp.h);
  //   points.push_back(p3);
  // }

  return lanelet::LineString3d(getId(), points);
}

double angleDiff(double a1, double a2)
{
  double a1_x = std::cos(a1);
  double a1_y = std::sin(a1);
  double a2_x = std::cos(a2);
  double a2_y = std::sin(a2);

  return std::fabs(std::acos(a1_x * a2_x + a1_y * a2_y));
}

std::pair<int, int> combineStraightLines(const VectorMap& vmap, int lid)
{
  auto line = vmap.findByKey(LineKey(lid));

  double angle_thresh = M_PI / 6;
  double base_angle = getLineAngle(vmap, line.lid);
  double current_angle = 0;
  double angle_diff = 0;

  std::pair<int, int> straight_line;
  straight_line.first = line.lid;
  straight_line.second = line.lid;

  while (line.lid != 0)
  {
    double current_angle = getLineAngle(vmap, line.lid);
    if (angleDiff(current_angle, base_angle) < angle_thresh)
    {
      straight_line.first = line.lid;
    }
    else
    {
      break;
    }
    line = vmap.findByKey(LineKey(line.blid));
  }

  line = vmap.findByKey(LineKey(lid));
  while (line.lid != 0)
  {
    double current_angle = getLineAngle(vmap, line.lid);
    if (angleDiff(current_angle, base_angle) < angle_thresh)
    {
      straight_line.second = line.lid;
    }
    else
    {
      break;
    }
    line = vmap.findByKey(LineKey(line.flid));
  }
  return straight_line;
}

lanelet::Lanelet convertCrosswalkToLanelet(const VectorMap& vmap, int area_id)
{
  auto area = vmap.findByKey(AreaKey(area_id));

  auto line = vmap.findByKey(LineKey(area.slid));

  std::pair<double, int> first_line, second_line;
  first_line.first = first_line.second = second_line.first, second_line.second = 0;

  std::unordered_set<long int> done;

  while (line.lid != 0)
  {
    double length = getLineLength(vmap, line.lid);

    if (length > first_line.first)
    {
      second_line = first_line;
      first_line.first = length;
      first_line.second = line.lid;
    }
    // else if (length > second_line.first)
    // {
    //   second_line.first = length;
    //   second_line.second = line.lid;
    // }
    line = vmap.findByKey(LineKey(line.flid));
  }
  auto first_lines = combineStraightLines(vmap, first_line.second);
  auto left_bound = convertToLineString(vmap, first_lines, done);

  line = vmap.findByKey(LineKey(area.slid));
  while (line.lid != 0)
  {
    if (exists(done, line.lid))
    {
      line = vmap.findByKey(LineKey(line.flid));
      continue;
    }
    double length = getLineLength(vmap, line.lid);

    if (length > second_line.first)
    {
      second_line.first = length;
      second_line.second = line.lid;
    }
    // else if (length > second_line.first)
    // {
    //   second_line.first = length;
    //   second_line.second = line.lid;
    // }
    line = vmap.findByKey(LineKey(line.flid));
  }
  auto second_lines = combineStraightLines(vmap, second_line.second);
  auto right_bound = convertToLineString(vmap, second_lines, done);

  lanelet::Lanelet llt(getId(), left_bound, right_bound);
  return llt;
}

void addCrossWalks(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap)
{
  std::vector<lanelet::Lanelet> crosswalks;
  auto vmap_crosswalks = vmap.findByFilter([](vector_map::CrossWalk cr) { return true; });

  for (const auto& cw : vmap_crosswalks)
  {
    if (cw.type != 0)
      continue;
    auto crosswalk_llt = convertCrosswalkToLanelet(vmap, cw.aid);
    crosswalk_llt.setAttribute("subtype", "crosswalk");
    lmap->add(crosswalk_llt);
    std::cout << crosswalk_llt << std::endl;
    std::cout << lmap->laneletLayer.get(crosswalk_llt.id()).attributes()["subtype"] << std::endl;
  }
}

void addIntersectionTags(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap)
{
  std::vector<lanelet::Polygon3d> intersections;
  auto vmap_intersections = vmap.findByFilter([](vector_map::CrossWalk cr) { return true; });
  // auto vmap_intersections = vmap.findByFilter([](vector_map::CrossRoad cr) { return true; });

  for (const auto& i : vmap_intersections)
  {
    if (i.type != 0)
      continue;
    auto poly = convertToPolygon(vmap, i.aid);
    intersections.push_back(poly);
    lmap->add(poly);
    std::cout << poly << std::endl;
  }

  for (auto& llt : lmap->laneletLayer)
  {
    auto llt_polygon2d = llt.polygon2d().basicPolygon();
    for (const auto& intersection : intersections)
    {
      auto intersection_polygon2d = lanelet::utils::to2D(intersection).basicPolygon();
      if (lanelet::geometry::distance(llt_polygon2d, intersection_polygon2d) < std::numeric_limits<double>::epsilon())
      {
        llt.attributes()["within_crosswalk"] = true;
      }
    }
  }
}

bool getNextLanelet(const VectorMap& vmap, const lanelet::LaneletMapPtr& lmap, const V2L& v2l_id, const L2V& l2v_id,
                    const lanelet::ConstLanelet llt, lanelet::ConstLanelet* next_llt)
{
  try
  {
    int lnid = l2v_id.at(llt.id());
    auto vmap_lane = vmap.findByKey(LaneKey(lnid));
    if (vmap_lane.lnid == 0)
      return false;
    if (vmap_lane.flid2 != 0)
      return false;
    auto vmap_next_lane = vmap.findByKey(LaneKey(vmap_lane.flid));
    if (vmap_next_lane.lnid == 0)
      return false;
    if (vmap_next_lane.blid2 != 0)
      return false;
    auto next_llt_id = v2l_id.at(vmap_next_lane.lnid);
    *next_llt = lmap->laneletLayer.get(next_llt_id);

    if (llt.attributeOr("within_crosswalk", false) != next_llt->attributeOr("within_crosswalk", false))
    {
      return false;
    }
  }
  catch (std::out_of_range& err)
  {
    std::cout << err.what() << std::endl;
    return false;
  }

  return true;
}

bool getPreviousLanelet(const VectorMap& vmap, const lanelet::LaneletMapPtr& lmap, const V2L& v2l_id, const L2V& l2v_id,
                        const lanelet::ConstLanelet llt, lanelet::ConstLanelet* prev_llt)
{
  try
  {
    int lnid = l2v_id.at(llt.id());
    auto vmap_lane = vmap.findByKey(LaneKey(lnid));
    if (vmap_lane.lnid == 0)
      return false;
    if (vmap_lane.blid2 != 0)
      return false;
    auto vmap_prev_lane = vmap.findByKey(LaneKey(vmap_lane.blid));
    if (vmap_prev_lane.lnid == 0)
      return false;
    if (vmap_prev_lane.flid2 != 0)
      return false;
    auto prev_llt_id = v2l_id.at(vmap_prev_lane.lnid);
    *prev_llt = lmap->laneletLayer.get(prev_llt_id);
    if (llt.attributeOr("within_crosswalk", false) != prev_llt->attributeOr("within_crosswalk", false))
    {
      return false;
    }
  }
  catch (std::out_of_range& err)
  {
    std::cout << err.what() << std::endl;
    return false;
  }

  return true;
}

lanelet::Lanelet combineLanelets(lanelet::Lanelets lanelets)
{
  std::vector<lanelet::Point3d> left_points, right_points;
  for (auto& llt : lanelets)
  {
    auto left_bound = llt.leftBound();
    auto right_bound = llt.rightBound();
    for (int i = 0; i < left_bound.size() - 1; i++)
    {
      left_points.push_back(left_bound[i]);
    }
    for (int i = 0; i < right_bound.size() - 1; i++)
    {
      right_points.push_back(right_bound[i]);
    }
  }

  if (!lanelets.empty())
  {
    auto end = lanelets.back();
    auto left_bound = end.leftBound();
    auto right_bound = end.rightBound();
    left_points.push_back(left_bound.back());
    right_points.push_back(right_bound.back());
  }

  lanelet::LineString3d left(getId(), left_points);
  lanelet::LineString3d right(getId(), right_points);

  SimplifyLineString(left);
  SimplifyLineString(right);

  lanelet::Lanelet lanelet(getId(), left, right);
  return lanelet;
}

void connectLanelets(const VectorMap& vmap, const lanelet::LaneletMapPtr& lmap, const V2L& v2l_id, const L2V& l2v_id)
{
  std::vector<lanelet::Lanelets> lanelets_array;
  std::unordered_set<lanelet::Id> done;

  lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);

  for (const auto& llt : lmap->laneletLayer)
  {
    if (exists(done, llt.id()))
    {
      continue;
    }
    done.insert(llt.id());
    if (llt.hasAttribute("subtype"))
    {
      std::cout << "crosswalk" << llt.id() << std::endl;
      new_map->add(llt);
      continue;
    }

    lanelet::Lanelets backward;
    lanelet::Lanelet prev_llt;
    lanelet::Lanelet current_llt = llt;
    while (getPreviousLanelet(vmap, lmap, v2l_id, l2v_id, current_llt, &prev_llt))
    {
      backward.push_back(prev_llt);
      done.insert(prev_llt.id());
      current_llt = prev_llt;
    }
    std::reverse(backward.begin(), backward.end());

    lanelet::Lanelets forward;
    lanelet::Lanelet next_llt;
    current_llt = llt;
    while (getNextLanelet(vmap, lmap, v2l_id, l2v_id, current_llt, &next_llt))
    {
      forward.push_back(next_llt);
      done.insert(next_llt.id());
      current_llt = next_llt;
    }

    lanelet::Lanelets connected_lanelets;
    connected_lanelets.insert(connected_lanelets.end(), backward.begin(), backward.end());
    connected_lanelets.push_back(llt);
    connected_lanelets.insert(connected_lanelets.end(), forward.begin(), forward.end());
    lanelets_array.push_back(connected_lanelets);
  }

  for (const auto& array : lanelets_array)
  {
    lanelet::Lanelet connected_lanelet = combineLanelets(array);
    connected_lanelet.attributes()["subtype"] = "road";
    new_map->add(connected_lanelet);
  }
  *lmap = std::move(*new_map);
}

void transformPoint(lanelet::LaneletMapPtr& lmap)
{
  for (auto& point : lmap->pointLayer)
  {
    double tmp_x = point.x();
    point.x() = point.y();
    point.y() = tmp_x;
    point.attributes()["local_x"] = point.x();
    point.attributes()["local_y"] = point.y();
  }
}

double getAngle(const lanelet::Point3d& p1, const lanelet::Point3d& p2)
{
  return std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
}

void SimplifyLineString(lanelet::LineString3d& line)
{
  std::vector<lanelet::Point3d> points;
  for (auto& pt : line)
  {
    points.push_back(pt);
  }
  std::unordered_set<lanelet::Id> remove_ids;

  auto prev_point = points.at(0);
  double prev_angle = 0;
  bool is_ready = true;
  for (auto& pt : points)
  {
    if (pt == prev_point)
      continue;

    if (!is_ready)
    {
      is_ready = true;
      prev_angle = getAngle(prev_point, pt);
      prev_point = pt;
      continue;
    }

    double current_angle = getAngle(prev_point, pt);

    if (angleDiff(prev_angle, current_angle) < M_PI / 360)
    {
      remove_ids.insert(prev_point.id());
    }
    else
    {
      prev_angle = current_angle;
    }
    prev_point = pt;
  }

  auto result =
      std::remove_if(points.begin(), points.end(), [&](lanelet::Point3d pt) { return exists(remove_ids, pt.id()); });
  points.erase(result, points.end());

  lanelet::LineString3d new_line(line.id(), points);
  line = new_line;
}

void SimplifyLineString(lanelet::LaneletMapPtr& lmap)
{
  for (auto& line : lmap->lineStringLayer)
  {
    SimplifyLineString(line);
  }
}

lanelet::Point3d convertSignalToPoint(const VectorMap& vmap, int id)
{
  auto signal = vmap.findByKey(SignalKey(id));
  auto signal_vector = vmap.findByKey(VectorKey(signal.vid));
  auto signal_point = vmap.findByKey(PointKey(signal_vector.pid));

  lanelet::Point3d pt(getId(), signal_point.bx, signal_point.ly, signal_point.h);

  switch (signal.type % 10)
  {
    case 1:
      pt.attributes()["color"] = "red";
      break;
    case 2:
      pt.attributes()["color"] = "green";
      break;
    case 3:
      pt.attributes()["color"] = "yellow";
      break;
  }

  if (signal.type > 20)
  {
    pt.attributes()["arrow"] = "left";
  }

  return pt;
}

lanelet::LineString3d getTrafficLightBase(const lanelet::LineString3d light_bulb)
{
  double min_height = std::numeric_limits<double>::max();
  double max_height = std::numeric_limits<double>::min();
  lanelet::Point3d pt_green;
  lanelet::Point3d pt_red;
  bool found_green = false;
  bool found_red = false;
  double radius = 0.8;
  for (const auto& pt : light_bulb)
  {
    if (pt.z() < min_height)
    {
      min_height = pt.z();
    }
    if (pt.z() > max_height)
    {
      max_height = pt.z();
    }
    std::string color = pt.attributeOr("color", "none");
    if (color.compare("green") == 0)
    {
      found_green = true;
      pt_green.x() = pt.x();
      pt_green.y() = pt.y();
    }
    if (color.compare("red") == 0)
    {
      found_red = true;
      pt_red.x() = pt.x();
      pt_red.y() = pt.y();
    }
  }
  if (!found_green || !found_red)
  {
    ROS_ERROR("could not found base line for traffic light!!!!");
    lanelet::LineString3d base_line;
    return base_line;
  }

  lanelet::BasicPoint3d direction;
  direction.x() = pt_red.x() - pt_green.x();
  direction.y() = pt_red.y() - pt_green.y();
  double length = std::hypot(direction.y(), direction.x());
  direction.x() = direction.x() / length;
  direction.y() = direction.y() / length;

  lanelet::Point3d p1(getId());
  p1.x() = pt_green.x() - direction.x() * radius;
  p1.y() = pt_green.y() - direction.y() * radius;
  p1.z() = min_height - radius;

  lanelet::Point3d p2(getId());
  p2.x() = pt_red.x() + direction.x() * radius;
  p2.y() = pt_red.y() + direction.y() * radius;
  p2.z() = min_height - radius;

  lanelet::LineString3d base_line(getId(), { p1, p2 });
  base_line.attributes()["height"] = max_height - min_height + radius * 2;
  return base_line;
}

void addTrafficLights(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap)
{
  auto vmap_signals = vmap.findByFilter([](vector_map::Signal s) { return true; });
  std::unordered_set<long int> done;
  for (const auto& signal : vmap_signals)
  {
    if (signal.type > 3 && signal.type < 20)
    {
      continue;
    }
    if (exists(done, signal.id))
    {
      continue;
    }
    const auto signals = vmap.findByFilter([signal](vector_map::Signal s) { return s.linkid == signal.linkid; });
    std::vector<lanelet::Point3d> points;
    for (const auto& s : signals)
    {
      done.insert(s.id);
      lanelet::Point3d pt = convertSignalToPoint(vmap, s.id);
      points.push_back(pt);
    }

    const auto vmap_stoplines = vmap.findByFilter([signals](vector_map::StopLine l) {
      for (const auto& s : signals)
      {
        if (l.tlid == s.vid)
          return true;
      }
      return false;
    });

    lanelet::LineString3d light_bulb(getId(), points);
    light_bulb.attributes()["type"] = "light_bulbs";
    lanelet::LineString3d base = getTrafficLightBase(light_bulb);
    // if(base.id() == 0)
    // {
    //   std::cout << "is invalid" << std::endl;
    //   continue;
    // }
    // light_bulb.attributes()["traffic_light_id"] = base.id();
    // for (const auto& vmap_stopline : vmap_stoplines)
    // {
      auto stop_line = convertToLineString(vmap, vmap_stopline.lid);
      stop_line.attributes()["type"] = "stop_line";
      auto tl = lanelet::autoware::AutowareTrafficLight::make(getId(), lanelet::AttributeMap(), { base }, stop_line,
                                                              { light_bulb });
      lmap->add(tl);
      std::cout << "ADDED" << std::endl;
    // }
  }
}
