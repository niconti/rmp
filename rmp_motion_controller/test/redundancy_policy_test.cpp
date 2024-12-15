#include <ros/ros.h>
#include <gtest/gtest.h>
#include <XmlRpcValue.h>
#include <Eigen/Dense>
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


TEST_F(MotionPolicyFixture, setConfig)
{
  XmlRpc::XmlRpcValue param;
  EXPECT_TRUE(node_ns.getParam("redundancy_policy", param));

  Eigen::VectorXd q_init(6);
  q_init << 0., 0., 0., 0., 0., 0.;

  rmp::RedundancyPolicy policy(q_init);
  policy.setConfig(param);
}


TEST_F(MotionPolicyFixture, setGoal)
{
  Eigen::VectorXd q_init(6);
  q_init << 0., 0., 0., 0., 0., 0.;
  rmp::RedundancyPolicy policy(q_init);

  Eigen::VectorXd q_goal(6);
  q_goal << 1., 1., 1., 1., 1., 1.;
  policy.setGoal(q_goal);
}


int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "redundancy_policy_test");
  return RUN_ALL_TESTS();
}