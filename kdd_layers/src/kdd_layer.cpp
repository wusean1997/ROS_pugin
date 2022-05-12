#include "kdd_layer.h"
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"//add_listener
#include "std_msgs/String.h"//add_listener


 
PLUGINLIB_EXPORT_CLASS(kdd_layer_namespace::KddLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;

namespace kdd_layer_namespace
{
 
KddLayer::KddLayer() {}

/**add listener**/
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
/**add listener**/

/**add listener**/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
/**add listener**/

void KddLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
 
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &KddLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}
 
 
void KddLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
 
void KddLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
 
  mark_x_ = origin_x + cos(origin_yaw)*2;
  mark_y_ = origin_y + sin(origin_yaw)*2;
 
  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}
 
void KddLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}
 
} // end namespace
