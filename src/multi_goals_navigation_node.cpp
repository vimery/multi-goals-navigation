#include <multi_goals_navigation/multi_navi.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_goals_navigation");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    
    multi_navi::MultiNavi multi_navi(buffer);

    ros::spin();
    return 0;
}
