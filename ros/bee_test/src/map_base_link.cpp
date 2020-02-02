#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
   tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link"));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "map_base_link_transformer");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/mavros/local_position/pose", 1000, &callback);

  ros::spin();
  return 0;
};
