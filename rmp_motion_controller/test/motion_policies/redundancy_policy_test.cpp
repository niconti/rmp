#include <ros/ros.h>
#include <gtest/gtest.h>
#include <XmlRpcValue.h>
#include "rmp_motion_controller/motion_policies/redundancy_policy.h"


class MotionPolicyFixture : public ::testing::Test {
protected:

  ros::NodeHandle node;
  ros::NodeHandle node_ns;

  MotionPolicyFixture() : node("~"), node_ns("")
  {

  }

  void SetUp() override
  {
    
  }

};


TEST_F(MotionPolicyFixture, config)
{
  XmlRpc::XmlRpcValue param;
  EXPECT_TRUE(node_ns.getParam("redundancy_policy", param));

  Eigen::VectorXd q_goal(6);
  q_goal << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  rmp::RedundancyPolicy policy(q_goal);
  policy.setConfig(param);
}


int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "redundancy_policy_test");
  return RUN_ALL_TESTS();
}