#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

class FusedOcTree : public octomap::OcTree {
public:
  explicit FusedOcTree(double resolution) : octomap::OcTree(resolution) {}

  octomap::OcTreeNode * createRoot() {
    if (this->root == nullptr) {
      this->root = new octomap::OcTreeNode();
      this->tree_size++;
      this->size_changed = true;
    }
    return this->root;
  }
};

using octomap::OcTree;
using octomap::OcTreeNode;

static bool isLeafNode(const OcTree & tree, const OcTreeNode * node, unsigned depth)
{
  if (node == nullptr || depth >= tree.getTreeDepth()) {
    return true;
  }
  for (unsigned i = 0; i < 8; ++i) {
    if (tree.nodeChildExists(node, i)) {
      return false;
    }
  }
  return true;
}

static float nodeValue(const OcTreeNode * node)
{
  return node ? node->getLogOdds() : 0.0f;
}

// a / b: nodes at `depth` in trees A / B (nullptr = unknown).
// outNode: already-created node of `out` at the same depth.
static void joinRecurs(
  const OcTree & A, const OcTree & B,
  const OcTreeNode * a, const OcTreeNode * b,
  unsigned depth,
  OcTree & out, OcTreeNode * outNode,
  float lmin, float lmax)
{
  const bool a_usable = isLeafNode(A, a, depth);
  const bool b_usable = isLeafNode(B, b, depth);

  if (a_usable && b_usable) {
    const float v = nodeValue(a) + nodeValue(b);
    outNode->setLogOdds(std::min(std::max(v, lmin), lmax));
    return;
  }

  // A coarser leaf's value applies to every child region below it; an inner
  // node with PARTIAL children (octomap creates only path children) only
  // contributes where a child actually exists (missing child = unknown).
  for (unsigned i = 0; i < 8; ++i) {
    const OcTreeNode * ca = nullptr;
    const OcTreeNode * cb = nullptr;
    if (a != nullptr) {
      if (a_usable) {
        ca = a;
      } else if (A.nodeChildExists(a, i)) {
        ca = A.getNodeChild(a, i);
      }
    }
    if (b != nullptr) {
      if (b_usable) {
        cb = b;
      } else if (B.nodeChildExists(b, i)) {
        cb = B.getNodeChild(b, i);
      }
    }
    if (ca == nullptr && cb == nullptr) {
      continue;  // unknown in both maps → stays unknown
    }
    OcTreeNode * out_child = out.createNodeChild(outNode, i);
    joinRecurs(A, B, ca, cb, depth + 1, out, out_child, lmin, lmax);
  }
}

static std::unique_ptr<FusedOcTree> fuseTrees(const OcTree & A, const OcTree & B)
{
  auto out = std::make_unique<FusedOcTree>(A.getResolution());
  const float lmin = A.getClampingThresMinLog();
  const float lmax = A.getClampingThresMaxLog();

  const OcTreeNode * ra = A.getRoot();
  const OcTreeNode * rb = B.getRoot();
  if (ra == nullptr && rb == nullptr) {
    return out;
  }

  OcTreeNode * root = out->createRoot();
  joinRecurs(A, B, ra, rb, 0, *out, root, lmin, lmax);

  out->updateInnerOccupancy();
  out->prune();
  return out;
}

class OctomapFusionNode : public rclcpp::Node
{
public:
  OctomapFusionNode()
  : Node("octomap_fusion_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    publish_free_space_ = declare_parameter<bool>("publish_free_space", true);
    occupancy_max_z_ = declare_parameter<double>("occupancy_max_z", 100.0);

    occ_color_.r = declare_parameter<double>("color.r", 0.0);
    occ_color_.g = declare_parameter<double>("color.g", 0.0);
    occ_color_.b = declare_parameter<double>("color.b", 1.0);
    occ_color_.a = declare_parameter<double>("color.a", 1.0);
    free_color_.r = declare_parameter<double>("color_free.r", 0.0);
    free_color_.g = declare_parameter<double>("color_free.g", 1.0);
    free_color_.b = declare_parameter<double>("color_free.b", 0.0);
    free_color_.a = declare_parameter<double>("color_free.a", 0.3);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();

    us_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_ultrasonic/octomap_full", latched,
      [this](octomap_msgs::msg::Octomap::SharedPtr msg) {this->octomapCb(msg, true);});
    lidar_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_lidar/octomap_full", latched,
      [this](octomap_msgs::msg::Octomap::SharedPtr msg) {this->octomapCb(msg, false);});

    binary_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/octomap_fused/octomap_binary", latched);
    full_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/octomap_fused/octomap_full", latched);
    occ_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/octomap_fused/occupied_cells_vis_array", latched);
    free_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/octomap_fused/free_cells_vis_array", latched);
    projected_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/octomap_fused/projected_map", latched);

    RCLCPP_INFO(get_logger(), "=== Octomap Fusion Node (Bayes log-odds) ===");
  }

private:
  void octomapCb(const octomap_msgs::msg::Octomap::SharedPtr msg, bool is_us)
  {
    std::shared_ptr<octomap::AbstractOcTree> tree(
      octomap_msgs::msgToMap(*msg),
      [](octomap::AbstractOcTree * p) {delete p;});

    if (!tree) {
      RCLCPP_WARN(get_logger(), "msgToMap failed (tree id '%s')", msg->id.c_str());
      return;
    }
    auto * oc = dynamic_cast<octomap::OcTree *>(tree.get());
    if (oc == nullptr) {
      RCLCPP_WARN(get_logger(), "Tree type '%s' is not an OcTree", msg->id.c_str());
      return;
    }
    if (is_us) {
      us_tree_ = oc;
      us_holder_ = tree;
    } else {
      lidar_tree_ = oc;
      lidar_holder_ = tree;
    }

    if (us_tree_ != nullptr && lidar_tree_ != nullptr) {
      fuseAndPublish();
    }
  }

  void fuseAndPublish()
  {
    if (us_tree_->getResolution() != lidar_tree_->getResolution()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Resolution mismatch: us=%.4f lidar=%.4f",
        us_tree_->getResolution(), lidar_tree_->getResolution());
      return;
    }

    auto fused = fuseTrees(*us_tree_, *lidar_tree_);

    // DEBUG: occupancy census of inputs and output
    {
      auto count_occ = [](const octomap::OcTree & t) {
          size_t n = 0;
          for (auto it = t.begin_leafs(); it != t.end_leafs(); ++it) {
            if (t.isNodeOccupied(*it)) {++n;}
          }
          return n;
        };
      const size_t us_occ = count_occ(*us_tree_);
      const size_t lidar_occ = count_occ(*lidar_tree_);
      const size_t fused_occ = count_occ(*fused);
      size_t checked = 0, missing = 0;
      for (auto it = us_tree_->begin_leafs(); it != us_tree_->end_leafs() && checked < 20; ++it) {
        if (!us_tree_->isNodeOccupied(*it)) {continue;}
        ++checked;
        const auto * n = fused->search(it.getX(), it.getY(), it.getZ());
        if (n == nullptr || !fused->isNodeOccupied(n)) {
          ++missing;
          RCLCPP_WARN(
            get_logger(), "US occupied cell missing in fused: (%.2f,%.2f,%.2f) size=%.2f l=%.2f found=%s",
            it.getX(), it.getY(), it.getZ(), it.getSize(), it->getLogOdds(),
            n ? (fused->isNodeOccupied(n) ? "occ" : "free") : "null");
        }
      }
      RCLCPP_INFO(
        get_logger(), "FUSE census: us=%zu(occ %zu) lidar=%zu(occ %zu) fused=%zu(occ %zu) missing=%zu/%zu",
        us_tree_->size(), us_occ, lidar_tree_->size(), lidar_occ,
        fused->size(), fused_occ, missing, checked);
    }

    const auto now = this->now();

    octomap_msgs::msg::Octomap binary_msg, full_msg;
    if (octomap_msgs::binaryMapToMsg(*fused, binary_msg)) {
      binary_msg.header.frame_id = frame_id_;
      binary_msg.header.stamp = now;
      binary_pub_->publish(binary_msg);
    }
    if (octomap_msgs::fullMapToMsg(*fused, full_msg)) {
      full_msg.header.frame_id = frame_id_;
      full_msg.header.stamp = now;
      full_pub_->publish(full_msg);
    }

    publishMarkers(*fused, now);
    publishProjectedMap(*fused, now);

    RCLCPP_DEBUG(
      get_logger(), "Fused tree: %zu nodes (us=%zu, lidar=%zu)",
      fused->size(), us_tree_->size(), lidar_tree_->size());
  }

  void publishMarkers(const FusedOcTree & tree, const rclcpp::Time & stamp)
  {
    const unsigned depth = tree.getTreeDepth();
    visualization_msgs::msg::MarkerArray occ, free;
    occ.markers.resize(depth + 1);
    free.markers.resize(depth + 1);

    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      const double half = it.getSize() / 2.0;
      if (it.getZ() - half > occupancy_max_z_) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = it.getX();
      p.y = it.getY();
      p.z = it.getZ();
      if (tree.isNodeOccupied(*it)) {
        occ.markers[it.getDepth()].points.push_back(p);
      } else if (publish_free_space_) {
        free.markers[it.getDepth()].points.push_back(p);
      }
    }

    for (size_t i = 0; i < occ.markers.size(); ++i) {
      finishMarker(occ.markers[i], i, tree, stamp, occ_color_);
      finishMarker(free.markers[i], i, tree, stamp, free_color_);
    }
    occ_marker_pub_->publish(occ);
    if (publish_free_space_) {
      free_marker_pub_->publish(free);
    }
  }

  void finishMarker(
    visualization_msgs::msg::Marker & m, size_t idx,
    const FusedOcTree & tree, const rclcpp::Time & stamp,
    const std_msgs::msg::ColorRGBA & color)
  {
    m.header.frame_id = frame_id_;
    m.header.stamp = stamp;
    m.ns = "map";
    m.id = static_cast<int>(idx);
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    const double size = tree.getNodeSize(static_cast<unsigned>(idx));
    m.scale.x = size;
    m.scale.y = size;
    m.scale.z = size;
    m.color = color;
    m.pose.orientation.w = 1.0;
    m.action = m.points.empty() ? visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;
  }

  void publishProjectedMap(const FusedOcTree & tree, const rclcpp::Time & stamp)
  {
    if (tree.size() == 0) {
      return;
    }
    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    const double res = tree.getResolution();

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.header.stamp = stamp;
    grid.info.resolution = static_cast<float>(res);
    grid.info.origin.position.x = min_x - res / 2.0;
    grid.info.origin.position.y = min_y - res / 2.0;
    grid.info.origin.orientation.w = 1.0;
    grid.info.width = static_cast<uint32_t>(std::ceil((max_x - min_x) / res)) + 1;
    grid.info.height = static_cast<uint32_t>(std::ceil((max_y - min_y) / res)) + 1;
    grid.data.assign(
      static_cast<size_t>(grid.info.width) * grid.info.height,
      static_cast<int8_t>(-1));

    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      const uint32_t cx = static_cast<uint32_t>(std::floor((it.getX() - min_x) / res));
      const uint32_t cy = static_cast<uint32_t>(std::floor((it.getY() - min_y) / res));
      if (cx >= grid.info.width || cy >= grid.info.height) {
        continue;
      }
      int8_t & cell = grid.data[cy * grid.info.width + cx];
      const int8_t v = tree.isNodeOccupied(*it) ? int8_t{100} : int8_t{0};
      // occupied (100) wins over free (0) wins over unknown (-1)
      if (cell < 0 || (cell == 0 && v == 100)) {
        cell = v;
      }
    }
    projected_map_pub_->publish(grid);
  }

  std::string frame_id_;
  bool publish_free_space_{true};
  double occupancy_max_z_{100.0};
  std_msgs::msg::ColorRGBA occ_color_, free_color_;

  octomap::OcTree * us_tree_{nullptr};
  octomap::OcTree * lidar_tree_{nullptr};
  std::shared_ptr<octomap::AbstractOcTree> us_holder_, lidar_holder_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr us_sub_, lidar_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_pub_, full_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occ_marker_pub_,
    free_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr projected_map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapFusionNode>());
  rclcpp::shutdown();
  return 0;
}
