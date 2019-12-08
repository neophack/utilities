
#include <vector_map_converter/vectormap2lanelet.hpp>

using VectorMap = vector_map::VectorMap;
using Node2Angle = std::unordered_map<int, double>;
using LaneKey = vector_map::Key<vector_map::Lane>;
using NodeKey = vector_map::Key<vector_map::Node>;
using PointKey = vector_map::Key<vector_map::Point>;
using DTLaneKey = vector_map::Key<vector_map::DTLane>;
using AreaKey = vector_map::Key<vector_map::Area>;
using LineKey = vector_map::Key<vector_map::Line>;
// using LaneKey = vector_map::Key<vector_map::Lane>;
// using LaneKey = vector_map::Key<vector_map::Lane>;
// using LaneKey = vector_map::Key<vector_map::Lane>;
// using LaneKey = vector_map::Key<vector_map::Lane>;

using lanelet::utils::getId;

void convertVectorMap2Lanelet2(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap)
{
  const auto node2angle = getNode2Angle(vmap);
  convertLanes(vmap, lmap, node2angle);
  addIntersectionTags(vmap, lmap);
  addCrossWalks(vmap, lmap);
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

  auto left_p1 = movePointByDirection(lanelet_point_bn, bn_angle + M_PI / 2, left_width);
  auto left_p2 = movePointByDirection(lanelet_point_fn, fn_angle + M_PI / 2, left_width);
  auto right_p1 = movePointByDirection(lanelet_point_bn, bn_angle - M_PI / 2, right_width);
  auto right_p2 = movePointByDirection(lanelet_point_fn, fn_angle - M_PI / 2, right_width);

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

void convertLanes(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap, const Node2Angle& node2angle)
{
  for (auto vmap_lane : vmap.findByFilter([](vector_map::Lane l) { return true; }))
  {
    const auto& lanelet = createLanelet(vmap, vmap_lane.lnid, node2angle);
    lmap->add(lanelet);
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

lanelet::LineString3d convertToLineString(const VectorMap& vmap, std::pair<int, int> straight_line)
{
  auto line = vmap.findByKey(LineKey(straight_line.first));
  std::vector<lanelet::Point3d> points;
  auto bp = vmap.findByKey(PointKey(line.bpid));
  lanelet::Point3d p1(getId(), bp.bx, bp.ly, bp.h);
  points.push_back(p1);

  while (line.lid != 0 && line.blid != straight_line.second)
  {
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

  while (line.lid != 0)
  {
    double length = getLineLength(vmap, line.lid);

    if (length > first_line.first)
    {
      second_line = first_line;
      first_line.first = length;
      first_line.second = line.lid;
    }
    else if (length > second_line.first)
    {
      second_line.first = length;
      second_line.second = line.lid;
    }
    line = vmap.findByKey(LineKey(line.flid));
  }

  auto first_lines = combineStraightLines(vmap, first_line.second);
  auto second_lines = combineStraightLines(vmap, second_line.second);

  auto left_bound = convertToLineString(vmap, first_lines);
  auto right_bound = convertToLineString(vmap, second_lines);
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
    crosswalks.push_back(crosswalk_llt);
    lmap->add(crosswalk_llt);
    std::cout << crosswalk_llt << std::endl;
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
        llt.attributes()["within_crosswalk"] = "yes";
      }
    }
  }
}

bool exists(std::unordered_set<lanelet::Id> set, lanelet::Id id)
{
  return set.find(id) != set.end();
}

bool getNextLanelet(const lanelet::ConstLanelet llt, lanelet::ConstLanelet* next_llt)
{
  lnid = l2v_id.at(llt.id());
  auto vmap_lane = vmap.findByKey(LaneKey(lnid));
  auto vmap_next_lane = vmap.findByKey(LaneKey(vmap_lane.flid));
  auto next_llt_id = v2l_id.at(vmap_next_lane.lnid);
  next_llt = lmap->laneletLayer.get(next_llt_id);
}


void connectLanelets()
{
  std::vector<lanelet::ConstLanelets> lanelets_array;
  std::unordered_set<lanelet::Id> done;

  for (const auto& llt : lmap->laneletLayer)
  {
    if (done.exists(llt.id))
    {
      continue;
    }
    done.insert(llt.id());

    lanelet::ConstLanelets backward;
    lanelet::ConstLanelet prev_llt;
    lanelet::ConstLanelet current_llt = llt;
    while (getPreviousLanelet(lmap, vmap, l2v_id, v2l_id, current_llt, &prev_llt))
    {
      backward.push_back(prev_llt);
      done.insert(prev_llt.id()) current_llt = prev_llt;
    }
    std::reverse(backward.begin(), backward.end());

    lanelet::ConstLanelets forward;
    lanelet::ConstLanelet next_llt;
    current_llt = llt;
    while (getNextLanelet(lmap, vmap, l2v_id, v2l_id, current_llt, &next_llt))
    {
      forward.push_back(next_llt);
      done.insert(next_llt.id()) current_llt = next_llt;
    }

    lanelet::ConstLanelets connected_lanelets;
    connected_lanelets.insert(connected_lanelets.end(), backward.begin(), backward.end());
    connected_lanelets.push_back(llt);
    connected_lanelets.insert(connected_lanelets.end(), forward.begin(), forward.end());
    lanelets_array(connected_lanelets);
  }

  for(const auto& array : lanelets_array)
  {
    lanelet::ConstLanelet connected_lanelet = combine_lanelets(array);
    lmap->add(connected_lanelet);
  }
}
