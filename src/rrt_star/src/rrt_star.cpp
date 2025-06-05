#include"rrt_star.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(rrtstar_planner::rrtstarPlannerROS, nav_core::BaseGlobalPlanner)
namespace rrtstar_planner
{
    rrtstarPlannerROS::rrtstarPlannerROS():costmap_(nullptr), initialized_(false){}
    rrtstarPlannerROS::rrtstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_ros_(costmap_ros),initialized_(false)
    {
        initialize(name, costmap_ros);
    }
    rrtstarPlannerROS::~rrtstarPlannerROS(){}

    void rrtstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            frame_id_ = costmap_ros_->getGlobalFrameID();

            resolution_ = costmap_->getResolution();

            //using hardcoded values for now, can be changed to parameters later
            search_radius_ = 0.5;  
            goal_radius_ = 0.1;  

            max_nodes_num_ = 1000000;  
            plan_time_out_ = 12.0;  
            
            epsilon_min_ = 0.001;  
            epsilon_max_ = 0.12;  

            path_point_spacing_ = 0.1;
            angle_difference_ = M_PI / 4;

            ROS_INFO("rrtstarPlannerROS initialized with name: %s", name.c_str());
            initialized_ = true;
        }else
        {
            ROS_ERROR("rrtstarPlannerROS has not been initialized, please call initialize() before using it.");
        }
    }
    
    bool rrtstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                                        const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan)
    {
        plan.clear();
        // make sure that there's no collision at the start and goal positions
        if (collision(start.pose.position.x, start.pose.position.y) ||
            collision(goal.pose.position.x, goal.pose.position.y))
        {
            ROS_ERROR("Start or goal position is in collision with obstacles.");
            return false;
        }

        marker_tree_.points.clear();
        marker_tree_2_.points.clear();

        std::vector<std::pair<double, double>> path;
        
        //first tree
        std::vector<Node> nodes;
        Node start_node;
        start_node.x = start.pose.position.x;
        start_node.y = start.pose.position.y;
        start_node.node_id = 0;
        start_node.parent_id = -1;
        start_node.cost = 0.0;
        nodes.push_back(start_node);
        
        //second tree
        std::vector<Node> nodes2;
        Node goal_node;
        goal_node.x = goal.pose.position.x;
        goal_node.y = goal.pose.position.y;
        goal_node.node_id = 0;
        goal_node.parent_id = -1;
        goal_node.cost = 0.0;
        nodes2.push_back(goal_node);

        std::pair<double, double> random_point;
        std::pair<double, double> new_point;// new point in the first tree
        std::pair<double, double> new_point2;// new point in the second tree

        Node connect_node_on_tree1;
        Node connect_node_on_tree2;
        bool is_connected = false;
        bool is_connected2 = false;

        Node nearest_node;

        unsigned int seed = 0;
        double start_time = ros::Time::now().toSec();
        while (ros::ok() && nodes.size() + nodes2.size() < max_nodes_num_ )
        {
            if( (ros::Time::now().toSec()-start_time) > plan_time_out_)
            {
                ROS_WARN("Plan time out after %.2f seconds.", plan_time_out_);
                return false;
            }

            // first tree
            for (int seed = 0; ros::ok(); seed++)
            {
               srand(ros::Time::now().toNSec() + seed);
               unsigned int rand_nu = rand()%10;

               if (rand_nu > 1)
               {
                 random_point = sampleFree();
               }else
               {
                 random_point.first = goal_node.x;
                 random_point.second = goal_node.y;
               }

               nearest_node = getNearest(nodes, random_point);
               new_point = steer(nearest_node.x, nearest_node.y, random_point.first, random_point.second);

               if(obstacleFree(nearest_node, new_point.first, new_point.second))
               {
                    Node new_node;
                    new_node.x = new_point.first;
                    new_node.y = new_point.second;
                    new_node.node_id = nodes.size();
                    new_node.parent_id = nearest_node.node_id;
                    new_node.cost = 0.0;

                    new_node =chooseParent(nearest_node, new_node, nodes);
                    nodes.push_back(new_node);
                    rewire(nodes, new_node);

                    geometry_msgs::Point p;
                    p.x = nodes[new_node.parent_id].x;
                    p.y = nodes[new_node.parent_id].y;
                    p.z = 0.0;
                    marker_tree_.points.push_back(p);
                    
                    p.x = new_node.x;
                    p.y = new_node.y;
                    p.z = 0.0;
                    marker_tree_.points.push_back(p);

                    if(nodes.size() % 10 == 0)
                    {
                        pubTreeMarker(marker_pub_,marker_tree_,1);
                    }

                    if(isConnect(new_node,nodes2,nodes, connect_node_on_tree2))
                    {
                        is_connected2 = true;
                    }
                    break;
               } 
            }

            if(is_connected2)
            {   
                ROS_INFO("Connected tree 1 to tree 2,it took %.2f seconds.", ros::Time::now().toSec() - start_time);
                getPathFromTree1ConnectTree2(nodes, nodes2, connect_node_on_tree2, plan);
                
                plan[0].pose.orientation = start.pose.orientation;
                plan[plan.size()-1].pose.orientation = goal.pose.orientation;

                nav_msgs::Path path_msg;
                path_msg.header.frame_id = frame_id_;
                path_msg.header.stamp = ros::Time::now();
                path_msg.poses = plan;
                plan_pub_.publish(path_msg);
                return true;
            }

            if (pointCircleCollision(new_point.first, new_point.second, goal.pose.position.x, goal.pose.position.y, goal_radius_))
            {
                ROS_INFO("Goal is reached by tree 1,it took %.2f seconds.", ros::Time::now().toSec() - start_time);
                getPathFromTree(nodes, nodes2, connect_node_on_tree2, plan, TREE1);
                
                plan[0].pose.orientation = start.pose.orientation;
                plan[plan.size()-1].pose.orientation = goal.pose.orientation;

                nav_msgs::Path path_msg;
                path_msg.header.frame_id = frame_id_;
                path_msg.header.stamp = ros::Time::now();
                path_msg.poses = plan;
                plan_pub_.publish(path_msg);
                return true;
            }

            //second tree
            random_point.first = new_point.first;
            random_point.second = new_point.second;

            for (int seed = 0; ros::ok(); seed++)
            {
                nearest_node = getNearest(nodes2, random_point);
                new_point2 = steer(nearest_node.x, nearest_node.y, random_point.first, random_point.second);
                if (obstacleFree(nearest_node, new_point2.first, new_point2.second))
                {
                    Node new_node2;
                    new_node2.x = new_point2.first;
                    new_node2.y = new_point2.second;
                    new_node2.node_id = nodes2.size();
                    new_node2.parent_id = nearest_node.node_id;
                    new_node2.cost = 0.0;

                    new_node2 = chooseParent(nearest_node, new_node2, nodes2);
                    nodes2.push_back(new_node2);
                    rewire(nodes2, new_node2);

                    geometry_msgs::Point p;
                    p.x = nodes2[new_node2.parent_id].x;
                    p.y = nodes2[new_node2.parent_id].y;
                    p.z = 0.0;
                    marker_tree_2_.points.push_back(p);
                    
                    p.x = new_node2.x;
                    p.y = new_node2.y;
                    p.z = 0.0;
                    marker_tree_2_.points.push_back(p);

                    if(nodes.size() % 10 == 0)
                    {
                        pubTreeMarker(marker_pub_,marker_tree_2_,1);
                    }

                    if(isConnect(new_node2,nodes,nodes2, connect_node_on_tree1))
                    {
                        is_connected = true;
                        break;
                    }
                
                }else
                {
                    srand(ros::Time::now().toNSec() + seed++);
                    unsigned int rand_nu = rand()%10;
                    if(rand_nu > 1) 
                    {
                        random_point = sampleFree(); 
                    }
                    else 
                    {
                        random_point.first = start.pose.position.x;
                        random_point.second = start.pose.position.y;
                    }
                }           
            }
            if (is_connected)
            {
                ROS_INFO("Connected tree 2 to tree 1,it took %.2f seconds.", ros::Time::now().toSec() - start_time);
                getPathFromTree1ConnectTree2(nodes, nodes2, connect_node_on_tree1, plan);
                
                plan[0].pose.orientation = start.pose.orientation;
                plan[plan.size()-1].pose.orientation = goal.pose.orientation;

                nav_msgs::Path path_msg;
                path_msg.header.frame_id = frame_id_;
                path_msg.header.stamp = ros::Time::now();
                path_msg.poses = plan;
                plan_pub_.publish(path_msg);
                return true;
            }
            if (pointCircleCollision(new_point2.first, new_point2.second, start.pose.position.x, start.pose.position.y, goal_radius_))
            {
                ROS_INFO("Goal is reached by tree 2,it took %.2f seconds.", ros::Time::now().toSec() - start_time);
                getPathFromTree(nodes, nodes2, connect_node_on_tree1, plan, TREE2);
                
                plan[0].pose.orientation = start.pose.orientation;
                plan[plan.size()-1].pose.orientation = goal.pose.orientation;

                nav_msgs::Path path_msg;
                path_msg.header.frame_id = frame_id_;
                path_msg.header.stamp = ros::Time::now();
                path_msg.poses = plan;
                plan_pub_.publish(path_msg);
                return true;
            }       
        }
        ROS_WARN("Failed to find a path after trying %d nodes in both trees.", max_nodes_num_);
        return false;
    }

    double rrtstarPlannerROS::normalizeAngle(double val,double min, double max)
    {
        double normalization = 0.0;

        if (val >= min)
            normalization = min + fmod((val - min), (max-min));
        else
            normalization = max - fmod((min - val), (max-min));

        return normalization;
    }

    void rrtstarPlannerROS::pubTreeMarker(ros::Publisher& marker_pub,visualization_msgs::Marker marker,int id)
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
        // in fact, y and z are not used for LINE_LIST, but we set them to avoid warnings
        marker.scale.y = 0.05; 
        marker.scale.z = 0.05; 

        marker.lifetime = ros::Duration();
        marker.frame_locked = false;

        marker_pub.publish(marker);
    }

    void rrtstarPlannerROS::optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan)
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

    bool rrtstarPlannerROS::pointCircleCollision(double x1, double y1, double x2, double y2, double radius)
    {
        double dist = distance(x1, y1, x2, y2);
        return dist <= radius;
    }

    double rrtstarPlannerROS::distance(double px1, double py1, double px2, double py2)
    {
        return sqrt(pow(px2 - px1, 2) + pow(py2 - py1, 2));
    }

    void rrtstarPlannerROS::insertPointForPath(std::vector< std::pair<double, double> >& path_in, double param)
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

    int rrtstarPlannerROS::optimizationPath(std::vector< std::pair<double, double> >& plan, double movement_angle_range)
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

    void rrtstarPlannerROS::cutPathPoint(std::vector< std::pair<double, double> >& plan)
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

    std::pair<double, double> rrtstarPlannerROS::sampleFree()
    {
        std::pair<double, double> random_point;
        unsigned int mx = 0,my = 0;
        double wx = 0.0,wy = 0.0;
        unsigned int map_size_x = costmap_->getSizeInCellsX();
        unsigned int map_size_y = costmap_->getSizeInCellsY();
        unsigned int seed = 0;
        while(ros::ok())
        {
        srand(ros::Time::now().toNSec() + seed++);
        mx = rand() % map_size_x;
        srand(ros::Time::now().toNSec() + seed++);
        my = rand() % map_size_y;
        if(costmap_->getCost(mx,my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            break;
        }
        costmap_->mapToWorld(mx,my,wx,wy);
        random_point.first = wx;
        random_point.second = wy;
        return random_point;
    }

    bool rrtstarPlannerROS::collision(double x, double y)
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

    bool rrtstarPlannerROS::isAroundFree(double wx, double wy)
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

  bool rrtstarPlannerROS::isConnect(Node new_node,std::vector< Node >& another_tree,std::vector< Node >& current_tree,Node& connect_node)
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

  Node rrtstarPlannerROS::getNearest(std::vector< Node > nodes, std::pair<double, double> p_rand)
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

  Node rrtstarPlannerROS::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
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

  void rrtstarPlannerROS::rewire(std::vector<Node>& nodes, Node newnode)
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

  std::pair<double, double> rrtstarPlannerROS::steer(double x1, double y1,double x2, double y2)
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

  bool rrtstarPlannerROS::obstacleFree(Node node_nearest, double px, double py)
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

}