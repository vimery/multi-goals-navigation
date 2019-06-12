#ifndef MULTI_NAVI_H_
#define MULTI_NAVI_H_

#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace multi_navi{
    
  class MultiNavi{
    public:
      /**
       * @brief Constructor
       */
      MultiNavi(tf2_ros::Buffer& tf);
      
      /**
       * @brief Destructor
       */
      virtual ~MultiNavi();
      
      /**
       * @brief Perform a navigation cycle
       * @param goals A list of goals to be expore
       * @return True if all goals have been reached, false otherwise
       */
      bool navigationCycle(std::vector<move_base_msgs::MoveBaseGoal>& goals);
    
    private:
      /**
       * @brief Perform a navigation
       * @param goal A goal where the target point is
       * @return True if the goal is reached, false otherwise
       */
      bool navigationOnce(move_base_msgs::MoveBaseGoal& goal);
      
      bool getGoals();
      
      /**
       * @brief sort the goals to generate a shortest path that pass through all goals
       */
      void sortGoals();

      /**
       * @brief compute the cost of a path given a start and a goal using make_plan in move_base package
       * @param start the start point of the path
       * @param goal the goal point of the path
       * @return cost of the path
       */
      int getPathCost(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);

      //set up goals buffer
      std::vector<move_base_msgs::MoveBaseGoal> goals_;
      
      tf2_ros::Buffer& tf_;
      // simple action client to send a goal
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
      // GetPlan service client to get a plan from a start point and a goal point
      ros::ServiceClient planServiceClient;

  };
}

#endif
