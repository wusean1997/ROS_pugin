#include <simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <iostream>
using namespace std;
//#include <std_msgs/Float32.h>
//#include <std_msgs/Float32MultiArray.h>

 
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
 
namespace simple_layer_namespace
{
 
  SimpleLayer::SimpleLayer() {}

  void SimpleLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    //subscribe function need to put in initial function
    /*
    //for Float32
    ros::NodeHandle ped_nh("pedestrian");
    pedestrian_sub_ = ped_nh.subscribe<std_msgs::Float32>("pedestrian_point", 1, boost::bind(&SimpleLayer::pedestrianCB, this, _1));*/
    
    /*
    //for Float32MultiArray
    ros::NodeHandle ped_nh("pedestrian");
    pedestrian_sub_ = ped_nh.subscribe<std_msgs::Float32MultiArray>("pedestrian_point", 1, boost::bind(&SimpleLayer::pedestrianCB, this, _1));
    */
    
    ros::NodeHandle ped_nh("pedestrian");
    pedestrian_sub_ = ped_nh.subscribe<pedsim_msgs::AgentStates>("/pedsim_simulator/simulated_agents", 1, &SimpleLayer::pedestrianCB, this);
    
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &SimpleLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
  }
  //add callback function detail
  /*
  //for Float32
  void SimpleLayer::pedestrianCB(const std_msgs::Float32::ConstPtr& msg)
  {
    point1 = msg->data;
    ROS_INFO("I get [%f]", point1);
  }
  */

  /*
  //for Float32MultiArray
  void SimpleLayer::pedestrianCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    point1 = msg->data.at(0);
    point2 = msg->data.at(1);
    point3 = msg->data.at(2);
    ROS_INFO("I get [%f],[%f],[%f]", point1,point2,point3);  
  }
  */
  
  void SimpleLayer::pedestrianCB(const pedsim_msgs::AgentStatesConstPtr& agents)
  { 
  
    size = agents->agent_states.size();
    for (uint actor = 0; actor< size ; actor++) {
        pedestrian[actor].x = agents->agent_states[actor].pose.position.x;
        pedestrian[actor].y = agents->agent_states[actor].pose.position.y;
        ROS_INFO("ID:[%i],Pose_x:[%f],Pose_y:[%f]", actor, pedestrian[actor].x, pedestrian[actor].y);
    }
  }
  


  void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                            double* min_y, double* max_x, double* max_y)
  {
    if (!enabled_)
      return;
    /*
    //for float var point1,point2,point3
    //mark_x_ = origin_x + point1 * cos(point3);
    //mark_y_ = origin_y + point2 * sin(point3);
    */

    /*
    //simple layer
    mark_x_ = 2.466065;
    mark_y_ = 1.453234;
    
    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);
    */
    
    for (uint actor = 0; actor< size ; actor++) {
      mark[actor].x = pedestrian[actor].x;
      mark[actor].y = pedestrian[actor].y;
    }
    
   

  }
 
  void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                            int max_j)
  {
    if (!enabled_)
      return;
    /*
    unsigned int mx;
    unsigned int my;
     if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    */
    for (uint actor = 0; actor< size ; actor++) {
      if(master_grid.worldToMap(mark[actor].x, mark[actor].y, map[actor].x, map[actor].y)){
      master_grid.setCost(map[actor].x, map[actor].y, LETHAL_OBSTACLE);
      }
    }
    
  }
 
} // end namespace

