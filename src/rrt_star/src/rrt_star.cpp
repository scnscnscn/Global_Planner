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
        if (this->collision(start.pose.position.x, start.pose.position.y) ||
            this->collision(goal.pose.position.x, goal.pose.position.y))
        {
            ROS_ERROR("Start or goal position is in collision with obstacles.");
            return false;
        }

        this->marker_tree_.points.clear();
        this->marker_tree_2_.points.clear();

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
                    this->marker_tree_.points.push_back(p);
                    
                    p.x = new_node.x;
                    p.y = new_node.y;
                    p.z = 0.0;
                    this->marker_tree_.points.push_back(p);

                    if(nodes.size() % 10 == 0)
                    {
                        this->pubTreeMarker(this->marker_pub_,this->marker_tree_,1);
                    }

                    if(this->isConnect(new_node,nodes2,nodes, connect_node_on_tree2))
                    {
                        is_connected2 = true;
                    }
                    break;
               } 
            }

            if(is_connected2)
            {   
                ROS_INFO("Connected tree 1 to tree 2,it took %.2f seconds.", ros::Time::now().toSec() - start_time);
                this->getPathFromTree1ConnectTree2(nodes, nodes2, connect_node_on_tree2, plan);
                
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
                    this->marker_tree_2_.points.push_back(p);
                    
                    p.x = new_node2.x;
                    p.y = new_node2.y;
                    p.z = 0.0;
                    this->marker_tree_2_.points.push_back(p);

                    if(nodes.size() % 10 == 0)
                    {
                        this->pubTreeMarker(this->marker_pub_,this->marker_tree_2_,1);
                    }

                    if(this->isConnect(new_node2,nodes,nodes2, connect_node_on_tree1))
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
                this->getPathFromTree1ConnectTree2(nodes, nodes2, connect_node_on_tree1, plan);
                
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
}