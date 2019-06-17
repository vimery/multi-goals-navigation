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
  
  int MultiNavi::getPathCost(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
  {
    // full file the goal
    nav_msgs::GetPlan srv;
    srv.request.goal = goal;
    srv.request.start = start;
    srv.request.tolerance = 0.5;
    
    if (planServiceClient.call(srv)) 
    {
      // get path from plan
      if (!srv.response.plan.poses.empty()) 
      {
        for (const geometry_msgs::PoseStamped &p : srv.response.plan.poses) 
        {
          ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
        }
      } else 
      {
        ROS_WARN("Got empty plan");
      }
    }
    return 0;
  }
  
  geometry_msgs::PoseStamped MultiNavi::getCurrentPose()
  {
    geometry_msgs::PoseStamped t;
    return t;
  }
  
  void MultiNavi::sortGoals()
  {
    int n = goals_.size();
    if (n == 0 && n == 1)
      return;
    geometry_msgs::PoseStamped currentPose = getCurrentPose();
    // the diagonal is the cost from currentPose to i
    // the lower triangle is cost from goal i to goal j
    int costs[n][n];
    for (int i=0; i < n; i++)
    {
      for (int j=i; j < n; j++)
      {
        if (i == j)
          costs[i][j] = getPathCost(currentPose, goals_[i].target_pose);
        else
        {
          costs[i][j] = getPathCost(goals_[i].target_pose, goals_[j].target_pose);
          costs[j][i] = costs[i][j];
        }
      }
    }
    // search for the shortest path
    std::vector<move_base_msgs::MoveBaseGoal> newgoals(goals_);
    goals_.clear();
    
  }
  
  int MultiNavi::getShortestPath(int current, int goals, double costs[N][N], double dis[M][N+1])
  {
    // 如果已经计算了距离，直接返回
    if (dis[goals][current] != 0) return dis[goals][current];
    // 如果只剩一个点，返回当前点到其距离
    if (count1(goals)) 
    {
      return dis[goals][current] = costs[current][locate(goals)];
    }
    int mincost = 100000;
   
    // 遍历所有的点，从1开始到N-1
    for (int i=0;i<N;i++)
    {
      // 在goals中
      if (goals&(1<<i))
      {
	      // 递归计算最短距离
	      double nextcost = getShortestPath(i, goals&(~(1<<i)), costs, dis);
	      // 初始点特殊处理
        double curcost = current == N ? costs[i][i] : costs[current][i];
        if (nextcost + curcost < mincost)
	      {
	        mincost = nextcost + curcost;
	      }
	    }
    }
    return dis[goals][current] = mincost;
  }

  int MultiNavi::getpow2(int n)
  {
    int p = 1;
    while (n > 0)
    {
      p = p << 1;
      n--;
    }
    return p;
  }

  // 计算goals中点的个数
  bool MultiNavi::count1(int s)
  {
    int count = 0;
    while (s > 0)
    {
      s = s & (s-1);
      count++;
    }
    return count == 1;
  }

  // goals中只有一个点时，得到那个点的位置
  int MultiNavi::locate(int s)
  {
    int loc = 0;
    while (s > 0)
    {
      if (s & 1) break;
      s = s >> 1;
      loc++;
    }
    return loc;
  }

}
