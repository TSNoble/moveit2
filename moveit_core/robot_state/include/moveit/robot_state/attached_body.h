/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Ioan Sucan, Tom Noble */

#pragma once

#include <moveit/robot_model/link_model.h>
#include <moveit/transforms/transforms.h>
#include <geometric_shapes/check_isometry.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <set>
#include <functional>

namespace moveit
{
namespace core
{
class AttachedBody;
typedef std::function<void(AttachedBody* body, bool attached)> AttachedBodyCallback;

/** @brief Object defining bodies that can be attached to robot links.
 *
 * This is useful when handling objects picked up by the robot. */
class AttachedBody
{
public:
  /** \brief Construct an attached body for a specified \e link.
   *
   * The name of this body is \e id and it consists of \e shapes that attach to the link by the transforms
   * \e shape_poses.
   * 
   * The set of links that are allowed to be touched by this object is specified by \e touch_links.
   * 
   * detach_posture may describe a detach motion for the gripper when placing the object.
   * 
   * The shape and subframe poses are relative to the \e pose, and \e pose is relative to the parent link. */
  AttachedBody(const LinkModel* parent, const std::string& name, const Eigen::Isometry3d& pose,
               const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
               const std::set<std::string>& touch_links, const trajectory_msgs::msg::JointTrajectory& detach_posture,
               const moveit::core::FixedTransformsMap& subframe_poses = moveit::core::FixedTransformsMap());

  /** \brief Construct an attached body with a specified \e parent body.
   *
   * The name of this body is \e id and it consists of \e shapes that attach to the link by the transforms
   * \e shape_poses.
   *
   * The body inherits its parent link, touch links and detach_posture from the root body in the hierarchy,
   *
   * The shape and subframe poses are relative to the \e pose, and \e pose is relative to the parent body. */
  AttachedBody(const AttachedBody* parent, const std::string& name, const Eigen::Isometry3d& pose,
               const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& shape_poses,
               const moveit::core::FixedTransformsMap& subframe_poses = moveit::core::FixedTransformsMap());
  
  ~AttachedBody() = default;

  /** \brief Get the name of the attached body */
  const std::string& getName() const;

  /** \brief Get the pose of the attached body relative to the parent link */
  const Eigen::Isometry3d& getPose() const;

  /** \brief Get the pose of the attached body, relative to the world */
  const Eigen::Isometry3d& getGlobalPose() const;

  /** \brief Get the name of the link this body is attached to */
  const std::string& getAttachedLinkName() const;

  /** \brief Get the model of the link this body is attached to */
  const LinkModel* getAttachedLink() const;

  /** \brief Get the name of the body that this body is attached to */
  const std::string getParentBodyName() const;

  /** \brief Get the body that this body is attached to */
  const AttachedBody* getParentBody() const;

  /** \brief Determines if this body is directly attached to a link */
  bool isRootBody() const;

  /** \brief Get the name of body in the hierarchy that is directly attached to a link */
  const std::string getRootBodyName() const;

  /** \brief Get the body in the hierarchy that is directly attached to a link */
  const AttachedBody* getRootBody() const;

  /** \brief Get the names of all direct child bodies of this body */
  const std::vector<std::string> getDirectChildBodyNames() const;

  /** \brief Get all direct child bodies of this body */
  const std::map<std::string, AttachedBody*> getDirectChildBodies() const;

  /** \brief Get the direct child body of this body with name `name` */
  const AttachedBody* getDirectChildBody(const std::string& name) const;

  /** \brief Get the names of all direct and indirect child bodies of this body */
  const std::vector<std::string> getDescendantBodyNames() const;

  /** \brief Get all direct and indirect child bodies of this body */
  const std::map<std::string, AttachedBody*> getDescendantBodies() const;

  /** \brief Get the direct or indirect child body of this body with name `name` */
  const AttachedBody* getDescendantBody(const std::string& name) const;
  
  /** \brief Get the shapes that make up this attached body */
  const std::vector<shapes::ShapeConstPtr>& getShapes() const;

  /** \brief Get the shape poses (the transforms to the shapes of this body, relative to the pose). The returned
   *  transforms are guaranteed to be valid isometries. */
  const EigenSTL::vector_Isometry3d& getShapePoses() const;

  /** \brief Get the links that the attached body is allowed to touch */
  const std::set<std::string>& getTouchLinks() const;

  /** \brief Return the posture that is necessary for the object to be released, (if any). This is useful for example
     when storing the configuration of a gripper holding an object */
  const trajectory_msgs::msg::JointTrajectory& getDetachPosture() const;

  /** \brief Get the fixed transforms (the transforms to the shapes of this body, relative to the link). The returned
   *  transforms are guaranteed to be valid isometries. */
  const EigenSTL::vector_Isometry3d& getShapePosesInLinkFrame() const;

  /** \brief Get subframes of this object (relative to the object pose). The returned transforms are guaranteed to be
   * valid isometries. */
  const moveit::core::FixedTransformsMap& getSubframes() const;

  /** \brief Get subframes of this object (in the world frame) */
  const moveit::core::FixedTransformsMap& getGlobalSubframeTransforms() const;

  /** \brief Set all subframes of this object.
   *
   * Use these to define points of interest on the object to plan with
   * (e.g. screwdriver/tip, kettle/spout, mug/base).
   */
  void setSubframeTransforms(const moveit::core::FixedTransformsMap& subframe_poses);

  /** \brief Get the fixed transform to a named subframe on this body (relative to the body's pose)
   *
   * The frame_name needs to have the object's name prepended (e.g. "screwdriver/tip" returns true if the object's
   * name is "screwdriver"). Returns an identity transform if frame_name is unknown (and set found to false).
   * The returned transform is guaranteed to be a valid isometry. */
  const Eigen::Isometry3d& getSubframeTransform(const std::string& frame_name, bool* found = nullptr) const;

  /** \brief Get the fixed transform to a named subframe on this body (relative to the robot link)
   *
   * The frame_name needs to have the object's name prepended (e.g. "screwdriver/tip" returns true if the object's
   * name is "screwdriver"). Returns an identity transform if frame_name is unknown (and set found to false).
   * The returned transform is guaranteed to be a valid isometry. */
  const Eigen::Isometry3d& getSubframeTransformInLinkFrame(const std::string& frame_name, bool* found = nullptr) const;

  /** \brief Get the fixed transform to a named subframe on this body, relative to the world frame.
   * The frame_name needs to have the object's name prepended (e.g. "screwdriver/tip" returns true if the object's
   * name is "screwdriver"). Returns an identity transform if frame_name is unknown (and set found to false).
   * The returned transform is guaranteed to be a valid isometry. */
  const Eigen::Isometry3d& getGlobalSubframeTransform(const std::string& frame_name, bool* found = nullptr) const;

  /** \brief Check whether a subframe of given @frame_name is present in this object.
   *
   * The frame_name needs to have the object's name prepended (e.g. "screwdriver/tip" returns true if the object's
   * name is "screwdriver"). */
  bool hasSubframeTransform(const std::string& frame_name) const;

  /** \brief Get the global transforms (in world frame) for the collision bodies. The returned transforms are
   *  guaranteed to be valid isometries. */
  const EigenSTL::vector_Isometry3d& getGlobalCollisionBodyTransforms() const;

  /** \brief Set the padding for the shapes of this attached object */
  void setPadding(double padding);

  /** \brief Set the scale for the shapes of this attached object */
  void setScale(double scale);

  /** \brief Recompute global_collision_body_transform given the transform of the parent link */
  void computeTransform(const Eigen::Isometry3d& parent_link_global_transform);

private:
  /** \brief For bodies attached directly to links, this is the link attached to.
  For bodies with parent bodies, this is the link attached to the root body. */
  const LinkModel* parent_link_model_;

  /** \brief The attached body that owns this one, if hierarchical.
  If attached directly to a link, this is a nullptr */
  const AttachedBody* parent_body_;

  /** \brief Any bodies which are attached directly to this body */
  std::map<std::string, AttachedBody*> child_bodies_;

  /** \brief string name for reference */
  std::string name_;

  /** \brief The transform from the parent link to the attached body's pose*/
  Eigen::Isometry3d pose_;

  /** \brief The transform from the model frame to the attached body's pose  */
  Eigen::Isometry3d global_pose_;

  /** \brief The geometries of the attached body */
  std::vector<shapes::ShapeConstPtr> shapes_;

  /** \brief The transforms from the object's pose to the object's geometries*/
  EigenSTL::vector_Isometry3d shape_poses_;

  /** \brief The transforms from the link to the object's geometries*/
  EigenSTL::vector_Isometry3d shape_poses_in_link_frame_;

  /** \brief The global transforms for the attached bodies (computed by forward kinematics) */
  EigenSTL::vector_Isometry3d global_collision_body_transforms_;

  /** \brief The set of links this body is allowed to touch */
  std::set<std::string> touch_links_;

  /** \brief Posture of links for releasing the object (if any). This is useful for example when storing
      the configuration of a gripper holding an object */
  trajectory_msgs::msg::JointTrajectory detach_posture_;

  /** \brief Transforms to subframes on the object, relative to the object's pose. */
  moveit::core::FixedTransformsMap subframe_poses_;

  /** \brief Transforms to subframes on the object, relative to the model frame. */
  moveit::core::FixedTransformsMap global_subframe_poses_;
};
}  // namespace core
}  // namespace moveit
