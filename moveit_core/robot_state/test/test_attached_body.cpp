/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Tom Noble */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <gtest/gtest.h>

class SingleLinkRobot : public testing::Test
{
protected:
  void SetUp() override
  {
    static const std::string URDF_XML = R"(
      <?xml version="1.0" ?>
      <robot name="single_link_robot">
      <link name="base_link">
        <inertial>
          <mass value="2.81"/>
          <origin rpy="0 0 0" xyz="0.0 0.0 .0"/>
          <inertia ixx="0.1" ixy="-0.2" ixz="0.5" iyy="-.09" iyz="1" izz="0.101"/>
        </inertial>
        <collision name="my_collision">
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <geometry>
            <box size="1 2 1" />
          </geometry>
        </collision>
        <visual>
          <origin rpy="0 0 0" xyz="0.0 0 0"/>
          <geometry>
            <box size="1 2 1" />
          </geometry>
        </visual>
      </link>
      </robot>
    )";

    static const std::string SRDF_XML = R"xml(
      <?xml version="1.0" ?>
      <robot name="single_link_robot">
      </robot>
      )xml";

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(URDF_XML);
    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initString(*urdf_model, SRDF_XML);
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();
  }

void TearDown() {
    robot_state_->setToDefaultValues();
}

protected:
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

};

TEST_F(SingleLinkRobot, TestTrue) {
    EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
