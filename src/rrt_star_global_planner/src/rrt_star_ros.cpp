/**
 * @file rrt_star_ros.cpp
 * @brief RRT* algorithm implementation for ROS global planner plugin
 */
#include <rrt_star_global_planner/rrt_star_ros.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <tf/tf.h>
#include <random>

PLUGINLIB_EXPORT_CLASS(RRTstar_planner::RRTstarPlannerROS, nav_core::BaseGlobalPlanner)

namespace RRTstar_planner
{
  RRTstarPlannerROS::RRTstarPlannerROS() :
    costmap_(nullptr),
    initialized_(false)
  {}
  RRTstarPlannerROS::RRTstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    costmap_ros_(costmap_ros),
    initialized_(false)
  {
    initialize(name, costmap_ros);
  }
  RRTstarPlannerROS::~RRTstarPlannerROS()
  {}

  /**
   * @brief Initialize the planner
   * 
   * @param name The name of the planner
   * @param costmap_ros The costmap ROS wrapper
   */
  void RRTstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            frame_id_ = costmap_ros_->getGlobalFrameID();

            resolution_ = costmap_->getResolution();

            search_radius_ = 0.5;  
            goal_radius_ = 0.1;  

            max_nodes_num_ = 1000000;  
            plan_time_out_ = 12.0;  
            
            epsilon_min_ = 0.001;  
            epsilon_max_ = 0.12;  

            path_point_spacing_ = 0.1;
            angle_difference_ = M_PI / 4;

            ROS_INFO("RRTstarPlannerROS initialized with name: %s", name.c_str());
            initialized_ = true;
        }else
        {
            ROS_ERROR("RRTstarPlannerROS has not been initialized, please call initialize() before using it.");
        }
  }

  /**
   * @brief Normalize angle to specified range
   * 
   * @param val Angle to normalize
   * @param min Minimum value of range
   * @param max Maximum value of range
   * @return Normalized angle within the specified range
   */
  double RRTstarPlannerROS::normalizeAngle(double val,double min, double max)
    {
        double normalization = 0.0;

        if (val >= min)
            normalization = min + fmod((val - min), (max-min));
        else
            normalization = max - fmod((min - val), (max-min));

        return normalization;
    }

  /**
   * @brief Publish tree marker for visualization
   * 
   * @param marker_pub ROS publisher for the marker
   * @param marker The marker to publish
   * @param id Marker ID
   */
  void RRTstarPlannerROS::pubTreeMarker(ros::Publisher& marker_pub,visualization_msgs::Marker marker,int id)
    {
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "marker_namespace";

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.scale.x = 0.05; 
        marker.scale.y = 0.05; 
        marker.scale.z = 0.05; 

        marker.lifetime = ros::Duration();
        marker.frame_locked = false;

        marker_pub.publish(marker);
    }

  /**
   * @brief Make a plan from start pose to goal pose
   * 
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The resulting plan (output)
   * @return true if a valid plan was found, false otherwise
   */
  bool RRTstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan)
{
    plan.clear();

    if(this->collision(start.pose.position.x, start.pose.position.y))
    {
        ROS_WARN("failed to get a path.start point is obstacle.");
        return false;
    }

    if(this->collision(goal.pose.position.x, goal.pose.position.y))
    {
        ROS_WARN("failed to get a path.goal point is obstacle.");
        return false;
    }

    this->marker_tree_.points.clear();
    this->marker_tree_2_.points.clear();
    plan.clear();
    std::vector< std::pair<double, double> > path;

    std::vector< Node > nodes; // First tree
    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0;
    start_node.parent_id = -1; // None parent node
    start_node.cost = 0.0;
    nodes.push_back(start_node);

    std::vector< Node > nodes_2; // Second tree
    Node goal_node;
    goal_node.x = goal.pose.position.x;
    goal_node.y = goal.pose.position.y;
    goal_node.node_id = 0;
    goal_node.parent_id = -1; // None parent node
    goal_node.cost = 0.0;
    nodes_2.push_back(goal_node);

    std::pair<double, double> p_rand; // Randomly sampled feasible point
    std::pair<double, double> p_new; // New node for first tree
    std::pair<double, double> p_new_2; // New node for second tree
    Node connect_node_on_tree1; // Node on first tree closest to second tree when trees connect
    Node connect_node_on_tree2; // Node on second tree closest to first tree when trees connect
    bool is_connect_to_tree1 = false;
    bool is_connect_to_tree2 = false;

    Node node_nearest;

    unsigned int seed = 0;
    double start_time = ros::Time::now().toSec();

    /**
     * @brief Lambda function to sample a random point
     */
    auto sampleRandomPoint = [&]() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 9);
        unsigned int rand_nu = dis(gen);
        if(rand_nu > 1) 
        {
            p_rand = sampleFree(); 
        }
        else 
        {
            p_rand.first = goal.pose.position.x;
            p_rand.second = goal.pose.position.y;
        }
    };

    /**
     * @brief Lambda function to expand a tree
     * 
     * @param currentTree The tree to expand
     * @param anotherTree The other tree to check for connections
     * @param pNew The new point to add to the tree (output)
     * @param marker Visualization marker
     * @param isConnected Flag indicating if trees are connected (output)
     * @param connectNode Connection node (output)
     * @param target Target pose
     * @return true if tree was expanded, false otherwise
     */
    auto expandTree = [&](std::vector<Node>& currentTree, std::vector<Node>& anotherTree, 
                          std::pair<double, double>& pNew, visualization_msgs::Marker& marker, 
                          bool& isConnected, Node& connectNode, const geometry_msgs::PoseStamped& target) {
        sampleRandomPoint();
        node_nearest = getNearest(currentTree, p_rand);
        pNew = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);
        if (obstacleFree(node_nearest, pNew.first, pNew.second))
        {
            Node newnode;
            newnode.x = pNew.first;
            newnode.y = pNew.second;
            newnode.node_id = currentTree.size();
            newnode.parent_id = node_nearest.node_id;
            newnode.cost = 0.0;

            newnode = chooseParent(node_nearest, newnode, currentTree);
            currentTree.push_back(newnode);
            rewire(currentTree, newnode);

            geometry_msgs::Point point_tem;
            point_tem.x = currentTree[newnode.parent_id].x;
            point_tem.y = currentTree[newnode.parent_id].y;
            point_tem.z = 0;
            marker.points.push_back(point_tem);

            point_tem.x = newnode.x;
            point_tem.y = newnode.y;
            point_tem.z = 0;
            marker.points.push_back(point_tem);

            if(currentTree.size() % 10 == 0)
            {
                this->pubTreeMarker(this->marker_pub_, marker, currentTree == nodes ? 1 : 2);
            }

            if(this->isConnect(newnode, anotherTree, currentTree, connectNode))
            {
                isConnected = true;
            }

            return true;
        }
        return false;
    };

    /**
     * @brief Lambda function to check if goal is reached
     * 
     * @param p Current point
     * @param target Target pose
     * @param tree Current tree
     * @param mode Plan mode
     * @return true if goal is reached, false otherwise
     */
    auto checkGoalReached = [&](const std::pair<double, double>& p, const geometry_msgs::PoseStamped& target, 
                                const std::vector<Node>& tree, GetPlanMode mode) {
        if (pointCircleCollision(p.first, p.second, target.pose.position.x , target.pose.position.y, goal_radius_) )
        {
            std::cout << (tree == nodes ? "First tree" : "Second tree") << " reached target, time used: " << ros::Time::now().toSec() - start_time << " seconds" << std::endl;
            getPathFromTree(const_cast<std::vector<Node>&>(nodes), const_cast<std::vector<Node>&>(nodes_2), const_cast<Node&>(tree.back()), plan, mode);
            plan[0].pose.orientation = start.pose.orientation;
            plan[plan.size()-1].pose.orientation = goal.pose.orientation;

            nav_msgs::Path path_pose;
            path_pose.header.frame_id = this->frame_id_;
            path_pose.header.stamp = ros::Time::now();
            path_pose.poses = plan;
            plan_pub_.publish(path_pose);
            return true;
        }
        return false;
    };

    /**
     * @brief Lambda function to check if trees are connected
     * 
     * @param isConnected Connection flag
     * @param mode Plan mode
     * @return true if trees are connected, false otherwise
     */
    auto checkTreesConnected = [&](bool isConnected, GetPlanMode mode) {
        if(isConnected)
        {
            std::cout << "Trees connected " << (mode == CONNECT1TO2 ? "1->2" : "2->1") << " time used: " << ros::Time::now().toSec() - start_time << " seconds" << std::endl;
            Node& connectNode = mode == CONNECT1TO2 ? connect_node_on_tree2 : connect_node_on_tree1;
            getPathFromTree(nodes, nodes_2, connectNode, plan, mode);
            plan[0].pose.orientation = start.pose.orientation;
            plan[plan.size()-1].pose.orientation = goal.pose.orientation;

            nav_msgs::Path path_pose;
            path_pose.header.frame_id = this->frame_id_;
            path_pose.header.stamp = ros::Time::now();
            path_pose.poses = plan;
            plan_pub_.publish(path_pose);
            return true;
        }
        return false;
    };

    while (ros::ok() && nodes.size() + nodes_2.size() < max_nodes_num_)
    {
        if( (ros::Time::now().toSec()-start_time) > plan_time_out_)
        {
            ROS_WARN("failed to get a path.time out.");
            return false;
        }

        if (expandTree(nodes, nodes_2, p_new, this->marker_tree_, is_connect_to_tree2, connect_node_on_tree2, goal))
        {
            if (checkTreesConnected(is_connect_to_tree2, CONNECT1TO2)) return true;
            if (checkGoalReached(p_new, goal, nodes, TREE1)) return true;
        }

        p_rand.first = p_new.first;
        p_rand.second = p_new.second;
        if (expandTree(nodes_2, nodes, p_new_2, this->marker_tree_2_, is_connect_to_tree1, connect_node_on_tree1, start))
        {
            if (checkTreesConnected(is_connect_to_tree1, CONNECT2TO1)) return true;
            if (checkGoalReached(p_new_2, start, nodes_2, TREE2)) return true;
        }
    }
    ROS_WARN("failed to get a path.");
    return false;
}
  
  /**
   * @brief Extract path from tree
   * 
   * @param tree1 First tree (from start)
   * @param tree2 Second tree (from goal)
   * @param connect_node Connection node
   * @param plan Resulting plan (output)
   * @param mode Plan extraction mode
   */
  void RRTstarPlannerROS::getPathFromTree(std::vector< Node >& tree1,
                                          std::vector< Node >& tree2,
                                          Node& connect_node,
                                          std::vector<geometry_msgs::PoseStamped>& plan,
                                          GetPlanMode mode)
  {
    std::pair<double, double> point;
    std::vector< std::pair<double, double> > path;
    Node current_node;

    if(mode == GetPlanMode::CONNECT1TO2 )
    {
      current_node = tree1.back();
    }

    if(GetPlanMode::TREE1)
    {
      current_node = tree1.back();
    }

    if(mode == GetPlanMode::CONNECT2TO1)
    {
      current_node = connect_node;
    }

    if(mode == GetPlanMode::TREE2)
    {
      current_node = tree1[0];
    }

    while (current_node.parent_id != tree1[0].parent_id)
    {
      point.first = current_node.x;
      point.second = current_node.y;
      path.insert(path.begin(), point); 
      current_node = tree1[current_node.parent_id];
    }

    if(mode == GetPlanMode::CONNECT1TO2)
    {
      current_node = connect_node;
    }

    if(mode == GetPlanMode::TREE1)
    {
      current_node = tree2[0];
    }

    if(mode == GetPlanMode::TREE2) 
    {
      current_node = tree2.back();
    }

    if(mode == GetPlanMode::CONNECT2TO1) 
    {
      current_node = tree2.back();
    }

    while (current_node.parent_id != tree2[0].parent_id)
    {
      point.first = current_node.x;
      point.second = current_node.y;
      path.push_back(point);
      current_node = tree2[current_node.parent_id];
    }

    point.first = tree1[0].x;
    point.second = tree1[0].y;
    path.insert(path.begin(), point);
    point.first = tree2[0].x;
    point.second = tree2[0].y;
    path.push_back(point); 

    cutPathPoint(path);

    insertPointForPath(path,this->path_point_spacing_);

    optimizationPath(path,this->angle_difference_);


    ros::Time plan_time = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    for (size_t i = 0; i < path.size(); i++)
    {
      pose.header.stamp = plan_time;
      pose.header.frame_id = this->frame_id_;
      pose.pose.position.x = path[i].first;
      pose.pose.position.y = path[i].second;
      plan.push_back(pose);
    }

    optimizationOrientation(plan);
  }


  /**
   * @brief Optimize orientation of path poses
   * 
   * @param plan Path to optimize (input/output)
   */
  void RRTstarPlannerROS::optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan)
    {
        size_t n = plan.size();
        if (n < 2) 
            return;

        for (size_t i = 0; i < n-1; ++i)
        {
            double dx = plan[i+1].pose.position.x - plan[i].pose.position.x;
            double dy = plan[i+1].pose.position.y - plan[i].pose.position.y;
            double angle = atan2(dy, dx);
            tf::Quaternion q = tf::createQuaternionFromYaw(angle);
            plan[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
        }

        // Ensure the last point has the same orientation as the second to last point
        if (n > 1) {
            plan[n-1].pose.orientation = plan[n-2].pose.orientation;
        }
    }

  /**
   * @brief Check if a point is within a circle
   * 
   * @param x1 X coordinate of point
   * @param y1 Y coordinate of point
   * @param x2 X coordinate of circle center
   * @param y2 Y Coordinate of circle center
   * @param radius Circle radius
   * @return true if point is within circle, false otherwise
   */
  bool RRTstarPlannerROS::pointCircleCollision(double x1, double y1, double x2, double y2, double radius)
    {
        double dist = distance(x1, y1, x2, y2);
        return dist <= radius;
    }

  /**
   * @brief Calculate Euclidean distance between two points
   * 
   * @param px1 X coordinate of first point
   * @param py1 Y coordinate of first point
   * @param px2 X coordinate of second point
   * @param py2 Y coordinate of second point
   * @return Distance between points
   */
  double RRTstarPlannerROS::distance(double px1, double py1, double px2, double py2)
    {
        return sqrt(pow(px2 - px1, 2) + pow(py2 - py1, 2));
    }

  /**
   * @brief Insert points along path to ensure proper spacing
   * 
   * @param path_in Input path
   * @param param Desired spacing between points
   */
  void RRTstarPlannerROS::insertPointForPath(std::vector< std::pair<double, double> >& path_in, double param)
    {
        std::vector< std::pair<double, double> > path_out;
        size_t num = path_in.size();
        std::pair<double, double> point;
        double pp_dist = param;

        if (num < 2)
        {
            ROS_WARN("Path has less than 2 points, no points will be inserted.");
            return;
        }

        for(size_t i = 0; i < num - 1; i++)
        {
            double theta = atan2(path_in[i+1].second - path_in[i].second,
                                path_in[i+1].first - path_in[i].first);

            // Calculate number of points to insert using rounding (distance/pp_dist + 0.5)
            // This ensures the number of points is rounded to the nearest integer
            size_t insert_size = static_cast<size_t>(distance(path_in[i+1].first, path_in[i+1].second,
                                                                path_in[i].first, path_in[i].second) / pp_dist + 0.5);

            for(size_t j = 0; j < insert_size; j++)
            {
                point.first = path_in[i].first + j * pp_dist * cos(theta);
                point.second = path_in[i].second + j * pp_dist * sin(theta);
                path_out.push_back(point);
            }
        }

        path_out.push_back(path_in.back());
        path_in.clear();
        path_in.resize(path_out.size());
        for(size_t i = 0; i < path_out.size(); i++)
        {
            path_in[i] = path_out[i];
        }
    }

  /**
   * @brief Optimize path by smoothing sharp turns
   * 
   * @param plan Path to optimize (input/output)
   * @param movement_angle_range Maximum allowed angle difference
   * @return Number of iterations performed
   */
  int RRTstarPlannerROS::optimizationPath(std::vector< std::pair<double, double> >& plan, double movement_angle_range)
    {
        if(plan.empty())
            return 0;
        size_t num = plan.size(); 
        double px, py, cx, cy, nx, ny, a_p, a_n;
        bool is_run = false;
        int ci = 0;
        
        for(ci = 0; ci < 1000; ci++)
        {
            is_run = false;
            
            for(size_t i = 1; i < num - 1; i++) 
            {
                px = plan[i-1].first;
                py = plan[i-1].second;

                cx = plan[i].first;
                cy = plan[i].second;

                nx = plan[i+1].first;
                ny = plan[i+1].second;

                a_p = normalizeAngle(atan2(cy-py, cx-px), 0, 2*M_PI);
                a_n = normalizeAngle(atan2(ny-cy, nx-cx), 0, 2*M_PI);

                // smooth the path if the angle difference is greater than the specified range
                if(std::max(a_p, a_n) - std::min(a_p, a_n) > movement_angle_range)
                {
                    plan[i].first = (px + nx)/2;
                    plan[i].second = (py + ny)/2;
                    is_run = true; 
                }
            }

            if(!is_run)
                return ci;
        }
        
        return ci; 
    }

  /**
   * @brief Check if a line between two points is free of obstacles
   * 
   * @param p1 First point
   * @param p2 Second point
   * @return true if line is free, false otherwise
   */
  bool RRTstarPlannerROS::isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2)
  {
   std::pair<double, double> ptmp;
    ptmp.first = 0.0;
    ptmp.second = 0.0;

    double dist = sqrt( (p2.second-p1.second) * (p2.second-p1.second) +
                        (p2.first-p1.first) * (p2.first-p1.first) );
    if (dist < this->resolution_)
    {
        return true;
    }
    else
    {
      int value = int(floor(dist/this->resolution_));
      double theta = atan2(p2.second - p1.second,
                           p2.first - p1.first);
      int n = 1;
      for (int i = 0;i < value; i++)
      {
        ptmp.first = p1.first + this->resolution_*cos(theta) * n;
        ptmp.second = p1.second + this->resolution_*sin(theta) * n;
        if (collision(ptmp.first, ptmp.second))
          return false;
        n++;
      }
      return true;
    }
  }

  /**
   * @brief Reduce path points by removing unnecessary intermediate points
   * 
   * @param plan Path to optimize (input/output)
   */
  void RRTstarPlannerROS::cutPathPoint(std::vector< std::pair<double, double> >& plan)
    {
        size_t current_index = 0;
        size_t check_index = current_index+2;
        while (ros::ok())
        {
            if (current_index >= plan.size() - 2)
            {
                break; // No more points to check
            }
            if (isLineFree(plan[current_index], plan[check_index]))
            {
                std::vector< std::pair<double, double> >::iterator it = plan.begin()+ static_cast<int>(current_index + 1) ;
                // Only one point between current_index and check_index
                if(check_index-current_index-1 == 1)
                {
                    plan.erase(it);
                }
                // More than one point between current_index and check_index
                else
                {
                    plan.erase(it,it+static_cast<int>(check_index-current_index-1));
                    check_index = current_index + 2;
                }
            }
            else
            {
                // using double pointer method to traverse the plan
                if(check_index < plan.size()-1 )
                check_index++;
                else
                {
                current_index++;
                check_index = current_index + 2;
                }
            }
        }
    }

  /**
   * @brief Sample a free point in the costmap
   * 
   * @return A randomly sampled collision-free point
   */
  std::pair<double, double> RRTstarPlannerROS::sampleFree()
{
    std::pair<double, double> random_point;
    unsigned int mx = 0,my = 0;
    double wx = 0.0,wy = 0.0;
    unsigned int map_size_x = costmap_->getSizeInCellsX();
    unsigned int map_size_y = costmap_->getSizeInCellsY();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(0, map_size_x - 1);
    std::uniform_int_distribution<> dis_y(0, map_size_y - 1);

    while(ros::ok())
    {
        mx = dis_x(gen);
        my = dis_y(gen);
        if(costmap_->getCost(mx,my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            break;
    }
    costmap_->mapToWorld(mx,my,wx,wy);
    random_point.first = wx;
    random_point.second = wy;
    return random_point;
}
  
  /**
   * @brief Check if a point is in collision with obstacles
   * 
   * @param x X coordinate
   * @param y Y coordinate
   * @return true if point is in collision, false otherwise
   */
  bool RRTstarPlannerROS::collision(double x, double y)
    {
        unsigned int mx,my;
        if(!costmap_->worldToMap(x,y,mx,my))
        return true;
        if ((mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
        return true;
        if (costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        return true;
        return false;
    }

  /**
   * @brief Check if the area around a point is free of obstacles
   * 
   * @param wx X coordinate in world frame
   * @param wy Y coordinate in world frame
   * @return true if surrounding area is free, false otherwise
   */
  bool RRTstarPlannerROS::isAroundFree(double wx, double wy)
    {
        unsigned int mx, my;
        if(!costmap_->worldToMap(wx,wy,mx,my))
        return false;
        if(mx <= 1 || my <= 1 || mx >= costmap_->getSizeInCellsX()-1 || my >= costmap_->getSizeInCellsY()-1)
        return false;
        int x,y;
        for(int i=-1;i<=1;i++)
        {
        for(int j=-1;j<=1;j++)
        {
            x = static_cast<int>(mx) + i;
            y = static_cast<int>(my) + j;
            if(costmap_->getCost(static_cast<unsigned int>(x),static_cast<unsigned int>(y)) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            return false;
        }
        }
        return true;
    }

  /**
   * @brief Check if a node can connect to another tree
   * 
   * @param new_node New node
   * @param another_tree The other tree to connect to
   * @param current_tree Current tree
   * @param connect_node Connection node (output)
   * @return true if trees can connect, false otherwise
   */
  bool RRTstarPlannerROS::isConnect(Node new_node,std::vector< Node >& another_tree,std::vector< Node >& current_tree,Node& connect_node)
  {
    size_t node_size = another_tree.size();
    double min_distance = 10000000.0;
    double dist = 0.0;
    size_t min_distance_index = 0;

    for(size_t i=0;i<node_size;i++)
    {
      dist = distance(new_node.x,new_node.y,another_tree[i].x,another_tree[i].y);
      if(dist < min_distance)
      {
        min_distance = dist;
        min_distance_index = i;
      }
    }

    dist = distance(new_node.x,new_node.y,another_tree[min_distance_index].x,another_tree[min_distance_index].y);

    if(dist < goal_radius_)
    {
      connect_node = another_tree[min_distance_index];
      Node newnode = another_tree[min_distance_index];
      newnode = chooseParent(current_tree.back(), newnode, current_tree); 
      current_tree.push_back(newnode);
      rewire(current_tree, newnode);
      return true;
    }
    return false;
  }

  /**
   * @brief Find the nearest node in a tree to a given point
   * 
   * @param nodes Tree nodes
   * @param p_rand Random point
   * @return Nearest node in the tree
   */
  Node RRTstarPlannerROS::getNearest(std::vector< Node > nodes, std::pair<double, double> p_rand)
  {
    double dist_min = distance(nodes[0].x, nodes[0].y, p_rand.first, p_rand.second);
    double dist_curr = 0;
    size_t index_min = 0;
    for (size_t i = 1; i < nodes.size(); i++)
    {
      dist_curr = distance(nodes[i].x, nodes[i].y, p_rand.first, p_rand.second);
      if (dist_curr < dist_min)
      {
        dist_min = dist_curr;
        index_min = i;
      }
    }
    return nodes[index_min];
  }

  /**
   * @brief Choose best parent node for a new node
   * 
   * @param nn Nearest node
   * @param newnode New node
   * @param nodes Tree nodes
   * @return New node with updated parent and cost
   */
  Node RRTstarPlannerROS::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
  {
    double dist_nnn = distance(nn.x, nn.y, newnode.x, newnode.y);
    for (size_t i = 0; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < search_radius_ &&
          nodes[i].cost + distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < nn.cost + dist_nnn &&
          obstacleFree(nodes[i], newnode.x, newnode.y))
      {
        nn = nodes[i];
      }
    }
    newnode.cost = nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y);
    if(!isAroundFree(newnode.x,newnode.y))
      newnode.cost += 0.3;
    newnode.parent_id = nn.node_id;
    return newnode;
  }

  /**
   * @brief Rewire tree to maintain optimality
   * 
   * @param nodes Tree nodes (input/output)
   * @param newnode Newly added node
   */
  void RRTstarPlannerROS::rewire(std::vector<Node>& nodes, Node newnode)
  {
    for (size_t i = 0; i < nodes.size(); i++)
    {
      Node& node = nodes[i];
      if (node != nodes[newnode.parent_id] &&
          distance(node.x, node.y, newnode.x, newnode.y) < search_radius_ &&
          newnode.cost + distance(node.x, node.y, newnode.x, newnode.y) < node.cost &&
          obstacleFree(node, newnode.x, newnode.y))
      {
        node.parent_id = newnode.node_id;
        node.cost = newnode.cost + distance(node.x, node.y, newnode.x, newnode.y);
        if(!isAroundFree(node.x,node.y))
          node.cost += 0.3;
      }
    }
  }

  /**
   * @brief Steer from one point toward another with limited distance
   * 
   * @param x1 Start x coordinate
   * @param y1 Start y coordinate
   * @param x2 Target x coordinate
   * @param y2 Target y coordinate
   * @return New point after steering
   */
  std::pair<double, double> RRTstarPlannerROS::steer(double x1, double y1,double x2, double y2)
  {
    std::pair<double, double> p_new;
    double dist = distance(x1, y1, x2, y2);
    if (dist < epsilon_max_ && dist > epsilon_min_)
    {
      p_new.first = x1;
      p_new.second = y1;
    }
    else
    {
      double theta = atan2(y2-y1, x2-x1);
      p_new.first = x1 + epsilon_max_*cos(theta);
      p_new.second = y1 + epsilon_max_*sin(theta);
    }
    return p_new;
  }

  /**
   * @brief Check if path between node and point is obstacle-free
   * 
   * @param node_nearest Starting node
   * @param px Target x coordinate
   * @param py Target y coordinate
   * @return true if path is collision-free, false otherwise
   */
  bool RRTstarPlannerROS::obstacleFree(Node node_nearest, double px, double py)
  {
    std::pair<double, double> p_n;
    p_n.first = 0.0;
    p_n.second = 0.0;

    double dist = distance(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution_)
    {
      if (collision(px, py))
        return false;
      else
        return true;
    }
    else
    {
      int value = int(floor(dist/resolution_));
      double theta = atan2(py - node_nearest.y, px - node_nearest.x);
      int n = 1;
      for (int i = 0;i < value; i++)
      {
        p_n.first = node_nearest.x + n*resolution_*cos(theta);
        p_n.second = node_nearest.y + n*resolution_*sin(theta);
        if (collision(p_n.first, p_n.second))
          return false;
        n++;
      }
      return true;
    }
  }
}; // RRTstar_planner namespace