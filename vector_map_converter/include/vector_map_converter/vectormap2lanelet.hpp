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

#include <vector_map/vector_map.h>

using V2L = std::map<int, lanelet::Id>;
using L2V = std::map<lanelet::Id, int>;

using VectorMap = vector_map::VectorMap;
using Node2Angle = std::unordered_map<int, double>;

void convertVectorMap2Lanelet2(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap);
lanelet::Lanelet createLanelet(const VectorMap& vmap, int lnid, const Node2Angle& node2angle);
void convertLanes(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap, const Node2Angle& node2angle, V2L* v2l_id,
                  L2V* l2v_id);

double getLineLength(const VectorMap& vmap, int line_id);
double getLineAngle(const VectorMap& vmap, int line_id);
lanelet::Point3d movePointByDirection(lanelet::Point3d p, double angle, double distance);
double getLaneAngle(const VectorMap& vmap, int lnid);
double getAngleAverage(const std::vector<double>& angles);

Node2Angle getNode2Angle(const VectorMap& vmap);

void addIntersectionTags(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap);

lanelet::Polygon3d convertToPolygon(const VectorMap& vmap, int area_id);
void addCrossWalks(const VectorMap& vmap, lanelet::LaneletMapPtr& lmap);
lanelet::Lanelet convertCrosswalkToLanelet(const VectorMap& vmap, int area_id);

lanelet::LineString3d convertToLineString(const VectorMap& vmap, int line_id);
lanelet::LineString3d convertToLineString(const VectorMap& vmap, std::pair<int, int> straight_line);
std::pair<int, int> combineStraightLines(const VectorMap& vmap, int lid);

void connectLanelets(const VectorMap& vmap, const lanelet::LaneletMapPtr& lmap, const V2L& v2l_id, const L2V& l2v_id);
lanelet::ConstLanelet combineLanelets(lanelet::ConstLanelets lanelets);
bool getNextLanelet(const VectorMap& vmap, const lanelet::LaneletMapPtr& lmap, const V2L& v2l_id, const L2V& l2v_id,
                    const lanelet::ConstLanelet llt, lanelet::ConstLanelet* next_llt);
bool getPreviousLanelet(const VectorMap& vmap, const lanelet::LaneletMapPtr& lmap, const V2L& v2l_id, const L2V& l2v_id,
                        const lanelet::ConstLanelet llt, lanelet::ConstLanelet* prev_llt);

void transformPoint(lanelet::LaneletMapPtr& lmap);

void SimplifyLineString(lanelet::LaneletMapPtr& lmap);
void SimplifyLineString(lanelet::LineString3d& line);
