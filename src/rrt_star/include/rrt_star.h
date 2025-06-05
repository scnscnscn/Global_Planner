#ifndef RRT_STAR_ROS_H
#define RRT_STAR_ROS_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <vector>
#include <visualization_msgs/Marker.h>

struct Node
{
  double x;// x coordinate of the node
  double y;// y coordinate of the node
  int node_id;// unique identifier for the node
	int parent_id;// identifier of the parent node
  double cost;// cost to reach this node from the start node
  
  bool operator ==(const Node& node) 
  {
    return (fabs(x - node.x) < 0.0001) && (fabs(y - node.y) < 0.0001) && (node_id == node.node_id) && (parent_id == node.parent_id) && (fabs(cost - node.cost) < 0.0001) ;
  }

  bool operator !=(const Node& node) 
  {
    if((fabs(x - node.x) > 0.0001) || (fabs(y - node.y) > 0.0001) || (node_id != node.node_id) || (parent_id != node.parent_id) || (fabs(cost - node.cost) > 0.0001))
      return true;
    else
      return false;
  }
}; 

enum GetPlanMode
{
  TREE1 = 1,
  TREE2 = 2,
  CONNECT1TO2 = 3,
  CONNECT2TO1 = 4,
};

namespace rrtstar_planner 
{

  class rrtstarPlannerROS : public nav_core::BaseGlobalPlanner 
  {

    public:
      rrtstarPlannerROS();
      rrtstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      ~rrtstarPlannerROS();

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

      void getPathFromTree1ConnectTree2(std::vector< Node >& tree1,
                                        std::vector< Node >& tree2,
                                        Node& connect_node,
                                        std::vector<geometry_msgs::PoseStamped>& plan);

      void getPathFromTree(std::vector< Node >& tree1,
                           std::vector< Node >& tree2,
                           Node& connect_node,
                           std::vector<geometry_msgs::PoseStamped>& plan,
                           GetPlanMode mode);

      double distance(double px1, double py1, double px2, double py2);

      std::pair<double, double> sampleFree(); 

      bool collision(double x, double y); 
      bool isAroundFree(double wx, double wy);
      bool isConnect(Node new_node,std::vector< Node >& another_tree,std::vector< Node >& current_tree,Node& connect_node);
      Node getNearest(std::vector<Node> nodes, std::pair<double, double> p_rand); 
      Node chooseParent(Node nn, Node newnode, std::vector<Node> nodes);

      void rewire(std::vector<Node>& nodes, Node newnode);

      std::pair<double, double> steer(double x1, double y1,double x2, double y2); 

      bool obstacleFree(Node node_nearest, double px, double py); 

      bool pointCircleCollision(double x1, double y1, double x2, double y2, double radius);

      void optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan);

      void insertPointForPath(std::vector< std::pair<double, double> >& pathin,double param);

      int optimizationPath(std::vector< std::pair<double, double> >& plan,double movement_angle_range = M_PI/4);

      bool isLineFree(const std::pair<double, double> p1,const std::pair<double, double> p2);

      void cutPathPoint(std::vector< std::pair<double, double> >& plan); 

      double inline normalizeAngle(double val,double min = -M_PI,double max = M_PI); 

      void pubTreeMarker(ros::Publisher& marker_pub,visualization_msgs::Marker marker,int id);
    protected:

      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      std::string frame_id_;
      ros::Publisher plan_pub_;

    private:

      visualization_msgs::Marker marker_tree_;
      visualization_msgs::Marker marker_tree_2_;
      ros::Publisher marker_pub_;

      size_t max_nodes_num_;
      double plan_time_out_;
      double search_radius_;
      double goal_radius_;
      double epsilon_min_;
      double epsilon_max_;

      double path_point_spacing_;
      double angle_difference_;

      double resolution_;
      bool initialized_;

  };
} // rrtstar_planner namespace
#endif