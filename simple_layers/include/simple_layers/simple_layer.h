#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <iostream>
//#include <std_msgs/Float32MultiArray.h>
using namespace std;

namespace simple_layer_namespace
{
 
  class SimpleLayer : public costmap_2d::Layer
  {
    public:
      SimpleLayer();
    
      virtual void onInitialize();
      virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                                double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    
    private:
      void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
      
      //declare subscriber callback function
      /*
      void pedestrianCB(const std_msgs::Float32::ConstPtr& msg);  
      void pedestrianCB(const std_msgs::Float32MultiArray::ConstPtr& msg);
      */
      void pedestrianCB(const pedsim_msgs::AgentStatesConstPtr& agents);
      
      double mark_x_, mark_y_;//simple_layer
      
      //add value to keep subscribe data
      /*
      float point1,point2,point3; //random variable input simple_layer
      */
      uint size;
      uint pedestrian_id;
      float pedestrian_pose_x,pedestrian_pose_y;
      
      struct pose{
        float x;
        float y;
      }pedestrian[];

      struct mark_pose{
        float x;
        float y;
      }mark[];

      struct worldtomap{
        uint x;
        uint y;
      }map[];
      //add pedestrian_sub_ to keep Subscriber structure
      ros::Subscriber pedestrian_sub_;
      dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  };
}
#endif

