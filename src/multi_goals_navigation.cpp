#include <multi_goals_navigation/multi_navi.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>



namespace multi_navi{

  MultiNavi::MultiNavi(tf2_ros::Buffer& tf):
  tf_(tf),
  ac_("move_base", true){
    
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    
    // init plan service
    planServiceClient = nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan", true);
    
    if (!planServiceClient)
    {
      ROS_FATAL("could not initialize get plan service from %s", planServiceClient.getService().c_str());
      exit(-1);
    }
    
    // receive goals
    if(!getGoals())
    {
      ROS_INFO("No goals to be traverse");
      exit(-1);
    }
    
    // sort goals to generate shortest path
    sortGoals();
    
    // traverse goals
    if(navigationCycle(goals_))
    {
      ROS_INFO("Succeed to traversing all goals");
    }
    else
    {
      ROS_INFO("traversing goals failed");
    }
  }
  
  MultiNavi::~MultiNavi()
  {
  
  }
  
  bool MultiNavi::navigationCycle(std::vector<move_base_msgs::MoveBaseGoal>& goals)
  {
    for (auto goal : goals)
    {
      if(!navigationOnce(goal))
      {
        return false;
      }
    }
    return true;
  }
  
  bool MultiNavi::navigationOnce(move_base_msgs::MoveBaseGoal& goal)
  {
    while(!ac_.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    ac_.sendGoal(goal);
    
    ac_.waitForResult();
    
    if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      return true;
    }
    else{
      return false;
    }
  }
  
  bool MultiNavi::getGoals()
  {
    for (int i = 0; i < 3; i++)
    {
        move_base_msgs::MoveBaseGoal goal;
        //set up the frame parameters
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        /* moving towards the goal*/
        goal.target_pose.pose.position.x =  0 + 3 - i;
        goal.target_pose.pose.position.y =  0;
        goal.target_pose.pose.position.z =  0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;
        
        goals_.push_back(goal);
    }
    return true;
  }
  
  int getPathCost(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
  {
    nav_msgs::GetPlan srv;
    srv.request.
    if (serviceClient.call(srv)) 
    {
      //srv.response.plan.poses 为保存结果的容器，遍历取出
      if (!srv.response.plan.poses.empty()) 
      {
        for (int i =0; i < srv.response.plan.poses)  
          forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) {
ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
}
}
else {
ROS_WARN("Got empty plan");
}
    return 0;
  }
  
  void MultiNavi::sortGoals()
  {
    if (goals_.size() == 0 && goals_.size() == 1)
      return;
    
  }
}
