#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void coordInit (std::string name){
    static tf::TransformBroadcaster outFrame;
    tf::Transform transform;
    tf::Quanternion q;
    transform.setOrigin ( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation ( q );
    outFrame.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", part_name));
}
