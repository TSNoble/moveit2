/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan, E. Gil Jones */

#include <planning_models/kinematic_model.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <limits>
#include <queue>
#include <ros/console.h>
#include <cmath>
#include <set>

/* ------------------------ KinematicModel ------------------------ */

planning_models::KinematicModel::KinematicModel(const boost::shared_ptr<const urdf::Model> &urdf_model,
                                                const boost::shared_ptr<const srdf::Model> &srdf_model)
{
  if (urdf_model->getRoot())
  {
    const urdf::Link *root = urdf_model->getRoot().get();
    buildModel(urdf_model, srdf_model, root->name);
  }
  else
    ROS_WARN("No root link found");
}

planning_models::KinematicModel::KinematicModel(const boost::shared_ptr<const urdf::Model> &urdf_model,
                                                const boost::shared_ptr<const srdf::Model> &srdf_model,
                                                const std::string &root_link)
{ 
  buildModel(urdf_model, srdf_model, root_link);
}

planning_models::KinematicModel::~KinematicModel(void)
{
  for (std::map<std::string, JointModelGroup*>::iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
    delete it->second;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    delete joint_model_vector_[i];
  for (std::size_t i = 0 ; i < link_model_vector_.size() ; ++i)
    delete link_model_vector_[i];
}

namespace planning_models
{
static bool orderJointsByIndex(const KinematicModel::JointModel *a, const KinematicModel::JointModel *b)
{
  return a->getTreeIndex() < b->getTreeIndex();
}

static bool orderLinksByIndex(const KinematicModel::LinkModel *a, const KinematicModel::LinkModel *b)
{
  return a->getTreeIndex() < b->getTreeIndex();
}
}

const std::string& planning_models::KinematicModel::getName(void) const
{
  return model_name_;
}

void planning_models::KinematicModel::buildModel(const boost::shared_ptr<const urdf::Model> &urdf_model,
                                                 const boost::shared_ptr<const srdf::Model> &srdf_model,
                                                 const std::string &root_link)
{ 
  root_joint_ = NULL;
  model_name_ = urdf_model->getName(); 
  if (urdf_model->getRoot())
  {  

    // construct a bidirectional graph that represents the URDF tree
    std::map<const urdf::Link*, std::map<const urdf::Link*, const urdf::Joint*> > graph;
    std::queue<const urdf::Link*> q;
    q.push(urdf_model->getRoot().get());
    while (!q.empty())
    {
      const urdf::Link *l = q.front();
      q.pop();
      for (unsigned int i = 0 ; i < l->child_links.size() ; ++i)
      {
        graph[l][l->child_links[i].get()] = graph[l->child_links[i].get()][l] = l->child_links[i]->parent_joint.get();
        q.push(l->child_links[i].get());
      }
    }

    // construct a tree from the graph such that the root is root_link
    class NewParentTree
    {
    public:
      NewParentTree(const std::map<const urdf::Link*, std::map<const urdf::Link*, const urdf::Joint*> > *graph) : graph_(graph)
      {
      }
      
      void constructTree(const urdf::Link *current, const urdf::Link *parent)
      {
        if (graph_->find(current) == graph_->end())
          return;
        const std::map<const urdf::Link*, const urdf::Joint*> &child = graph_->at(current);
        for (std::map<const urdf::Link*, const urdf::Joint*>::const_iterator it = child.begin() ; it != child.end() ; ++it)
          if (it->first != parent)
          {
            constructTree(it->first, current);
            parent_map_[it->first] = std::make_pair(current, it->second);
            child_map_[current].push_back(it->first);
          }
      }

      const std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> >& getParentMap(void) const
      {
        return parent_map_;
      }
      
      const std::map<const urdf::Link*, std::vector<const urdf::Link*> >& getChildMap(void) const
      {
        return child_map_;
      }
      
    private:
      const std::map<const urdf::Link*, std::map<const urdf::Link*, const urdf::Joint*> > *graph_;
      std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > parent_map_;
      std::map<const urdf::Link*, std::vector<const urdf::Link*> > child_map_;
    };
    
    NewParentTree npt(&graph);
    const urdf::Link *root_link_ptr = urdf_model->getLink(root_link).get();
    if (root_link_ptr)
    {
      npt.constructTree(root_link_ptr, NULL);
      model_frame_ = root_link;
      root_joint_ = buildRecursive(NULL, root_link_ptr, npt.getParentMap(), npt.getChildMap(), srdf_model->getVirtualJoints());    

      root_link_ = link_model_map_[root_link];
      buildMimic(urdf_model);      
      
      // construct additional additional maps for easy access by name
      variable_count_ = 0;
      std::vector<JointModel*> later;
      for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
      {
        const std::vector<std::string> &name_order = joint_model_vector_[i]->getVariableNames();
        for (std::size_t j = 0 ; j < name_order.size() ; ++j)
          joint_model_vector_[i]->getVariableBounds(name_order[j], variable_bounds_[name_order[j]]);
        if (joint_model_vector_[i]->mimic_ == NULL)
        {
          // compute index map
          if (name_order.size() > 0)
          {
            for (std::size_t j = 0; j < name_order.size(); ++j)
            {
              joint_variables_index_map_[name_order[j]] = variable_count_ + j;
              active_dof_names_.push_back(name_order[j]);
            }
            joint_variables_index_map_[joint_model_vector_[i]->getName()] = variable_count_;
            
            // compute variable count
            variable_count_ += joint_model_vector_[i]->getVariableCount();
          }
        }
        else
          later.push_back(joint_model_vector_[i]);
      }
      
      for (std::size_t i = 0 ; i < later.size() ; ++i)
      {
        const std::vector<std::string>& name_order = later[i]->getVariableNames();
        const std::vector<std::string>& mim_name_order = later[i]->mimic_->getVariableNames();
        for (std::size_t j = 0; j < name_order.size(); ++j)
          joint_variables_index_map_[name_order[j]] = joint_variables_index_map_[mim_name_order[j]];
        joint_variables_index_map_[later[i]->getName()] = joint_variables_index_map_[later[i]->mimic_->getName()];
      }
      
      // build groups
      buildGroups(srdf_model->getGroups());
      
      // copy the default states to the groups
      const std::vector<srdf::Model::GroupState> &ds = srdf_model->getGroupStates();
      for (std::size_t i = 0 ; i < ds.size() ; ++i)
      {
        std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.find(ds[i].group_);
        if (it != joint_model_group_map_.end())
          for (std::map<std::string, std::vector<double> >::const_iterator jt = ds[i].joint_values_.begin() ; jt != ds[i].joint_values_.end() ; ++jt)
          {
            const JointModel* jm = it->second->getJointModel(jt->first);
            if (jm)
            {
              const std::vector<std::string> &vn = jm->getVariableNames();
              if (vn.size() == jt->second.size())
                for (std::size_t j = 0 ; j < vn.size() ; ++j)
                  it->second->default_states_[ds[i].name_][vn[j]] = jt->second[j];
              else
                ROS_ERROR("The model for joint '%s' requires %d variable values, but only %d variable values were supplied in default state '%s' for group '%s'",
                          jt->first.c_str(), (int)vn.size(), (int)jt->second.size(), ds[i].name_.c_str(), it->first.c_str());
            }
            else
              ROS_ERROR("Group state '%s' specifies value for joint '%s', but that joint is not part of group '%s'", ds[i].name_.c_str(),
                        jt->first.c_str(), it->first.c_str());
          }
        else
          ROS_ERROR("Group state '%s' specified for group '%s', but that group does not exist", ds[i].name_.c_str(), ds[i].group_.c_str());
      }

      std::stringstream ss;
      printModelInfo(ss);

      ROS_DEBUG("%s", ss.str().c_str());
    }
    else
      ROS_ERROR("Link '%s' (to be used as root) was not found in model '%s'. Cannot construct model", root_link.c_str(), model_name_.c_str());
  }
  else
    ROS_WARN("No root link found");
}

void planning_models::KinematicModel::buildMimic(const boost::shared_ptr<const urdf::Model> &urdf_model)
{
  // compute mimic joints
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const urdf::Joint *jm = urdf_model->getJoint(joint_model_vector_[i]->getName()).get();
    if (jm)
      if (jm->mimic)
      {
        joint_model_vector_[i]->mimic_offset_ = jm->mimic->offset;
        joint_model_vector_[i]->mimic_factor_ = jm->mimic->multiplier;
        std::map<std::string, JointModel*>::const_iterator jit = joint_model_map_.find(jm->mimic->joint_name);
        if (jit != joint_model_map_.end())
        {
          if (joint_model_vector_[i]->getVariableCount() == jit->second->getVariableCount())
            joint_model_vector_[i]->mimic_ = jit->second;
          else
            ROS_ERROR("Join '%s' cannot mimic joint '%s' because they have different number of DOF",
                      joint_model_vector_[i]->getName().c_str(), jm->mimic->joint_name.c_str());
        }
        else
          ROS_ERROR("Joint '%s' cannot mimic unknown joint '%s'", joint_model_vector_[i]->getName().c_str(), jm->mimic->joint_name.c_str());
      }
  }
  // in case we have a joint that mimics a joint that already mimics another joint, we can simplify things:
  bool change = true;
  while (change)
  {
    change = false;
    for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
      if (joint_model_vector_[i]->mimic_)
        if (joint_model_vector_[i]->mimic_->mimic_)
        {
          joint_model_vector_[i]->mimic_ = joint_model_vector_[i]->mimic_->mimic_;
          joint_model_vector_[i]->mimic_offset_ += joint_model_vector_[i]->mimic_factor_ * joint_model_vector_[i]->mimic_->mimic_offset_;
          joint_model_vector_[i]->mimic_factor_ *= joint_model_vector_[i]->mimic_->mimic_factor_;
          change = true;
        }
  }
  // build mimic requests
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_vector_[i]->mimic_ == joint_model_vector_[i])
    {
      joint_model_vector_[i]->mimic_ = NULL;
      ROS_ERROR("Joint '%s' mimicks itself. This is not allowed.", joint_model_vector_[i]->getName().c_str());
    }
    else
      if (joint_model_vector_[i]->mimic_)
        joint_model_vector_[i]->mimic_->mimic_requests_.push_back(joint_model_vector_[i]);
}

bool planning_models::KinematicModel::hasJointModelGroup(const std::string &name) const
{
  return joint_model_group_map_.find(name) != joint_model_group_map_.end();
}

const planning_models::KinematicModel::JointModelGroup* planning_models::KinematicModel::getJointModelGroup(const std::string& name) const
{
  std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.find(name);
  if (it == joint_model_group_map_.end())
  {
    ROS_ERROR_STREAM("Group '" << name << "' not found in model " << model_name_);
    return NULL;
  }
  return it->second;
}

void planning_models::KinematicModel::buildGroups(const std::vector<srdf::Model::Group>& group_configs)
{
  //the only thing tricky is dealing with subgroups
  std::vector<bool> processed(group_configs.size(), false);

  bool added = true;
  while (added)
  {
    added = false;

    //going to make passes until we can't do anything else
    for(unsigned int i = 0 ; i < group_configs.size() ; ++i)
      if(!processed[i])
      {
        //if we haven't processed, check and see if the dependencies are met yet
        bool all_subgroups_added = true;
        for(unsigned int j = 0; j < group_configs[i].subgroups_.size(); ++j)
          if (joint_model_group_map_.find(group_configs[i].subgroups_[j]) == joint_model_group_map_.end())
          {
            all_subgroups_added = false;
            break;
          }
        if (all_subgroups_added)
        {
          //only get one chance to do it right
          if (addJointModelGroup(group_configs[i]))
          {
            processed[i] = true;
            added = true;
          }
          else
            ROS_WARN_STREAM("Failed to add group '" << group_configs[i].name_ << "'");
        }
      }
  }

  for (unsigned int i = 0 ; i < processed.size() ; ++i)      
    if (!processed[i])
      ROS_WARN_STREAM("Could not process group '" << group_configs[i].name_ << "' due to unmet subgroup dependencies");
  
  // compute subgroups  
  JointModelGroup *largest_dim_group = NULL;
  for (std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end(); ++it)
  {
    JointModelGroup *jmg = it->second;
    if (!largest_dim_group || largest_dim_group->getVariableCount() < jmg->getVariableCount())
      largest_dim_group = jmg;
    
    jmg->subgroup_names_.clear();   
    jmg->disjoint_subgroup_names_.clear();
    std::set<const JointModel*> joints(jmg->getJointModels().begin(), jmg->getJointModels().end());
    for (std::map<std::string, JointModelGroup*>::const_iterator jt = joint_model_group_map_.begin() ; jt != joint_model_group_map_.end(); ++jt)
      if (jt->first != it->first)
      {
        bool ok = true;
        JointModelGroup *sub_jmg = jt->second;
        const std::vector<const JointModel*> &sub_joints = sub_jmg->getJointModels();
        for (std::size_t k = 0 ; k < sub_joints.size() ; ++k)
          if (joints.find(sub_joints[k]) == joints.end())
          {
            ok = false;
            break;
          }
        if (ok)
          jmg->subgroup_names_.push_back(sub_jmg->getName());
      }
  }
  
  if (largest_dim_group)
  {
    // add the first vertex to the MST
    std::set<JointModelGroup*> connected;
    connected.insert(largest_dim_group);
    bool done = false;
    while (!done)
    {
      JointModelGroup *from = NULL;
      JointModelGroup *to = NULL;
      unsigned int ft_weight = 0;
      for (std::set<JointModelGroup*>::iterator it = connected.begin() ; it != connected.end() ; ++it)
      {
        const std::vector<std::string> &sg = (*it)->subgroup_names_;
        const std::vector<std::string> &dsg = (*it)->disjoint_subgroup_names_;
        for (std::size_t i = 0 ; i < sg.size() ; ++i)
        {
          JointModelGroup *child = joint_model_group_map_.at(sg[i]);
          if (connected.find(child) != connected.end())
            continue;
          unsigned int weight = abs(child->getVariableCount() - (*it)->getVariableCount());
          if (to == NULL || weight < ft_weight)
          {
            // check if this new potential group is disjoint from the rest of the added groups
            bool ok = true;
            std::set<const JointModel*> cj(child->getJointModels().begin(), child->getJointModels().end());
            for (std::size_t k = 0 ; ok && k < dsg.size() ; ++k)
            {      
              const std::vector<const JointModel*> &nbh = joint_model_group_map_.at(dsg[k])->getJointModels();
              for (std::size_t nj = 0 ; ok && nj < nbh.size() ; ++nj)
                if (cj.find(nbh[nj]) != cj.end())
                  ok = false;
            }
            if (ok) // add only disjoint groups
            {
              from = *it;
              to = child;
              ft_weight = weight;
            }
          }
        }
      }
      if (to)
      {
        from->disjoint_subgroup_names_.push_back(to->getName());
        connected.insert(to);
      }
      else
        done = true;
    }
  }
}

void planning_models::KinematicModel::removeJointModelGroup(const std::string& group)
{
  if (getJointModelGroup(group))
  {
    delete joint_model_group_map_[group];
    joint_model_group_map_.erase(group);
    joint_model_group_config_map_.erase(group);
  }
}

bool planning_models::KinematicModel::addJointModelGroup(const srdf::Model::Group& gc)
{
  if (joint_model_group_map_.find(gc.name_) != joint_model_group_map_.end())
  {
    ROS_WARN_STREAM("A group named '" << gc.name_ <<"' already exists. Not adding.");
    return false;
  }

  std::set<const JointModel*> jset;

  // add joints from chains
  for (std::size_t i = 0 ; i < gc.chains_.size() ; ++i)
  {
    const LinkModel* base_link = getLinkModel(gc.chains_[i].first);
    const LinkModel* tip_link = getLinkModel(gc.chains_[i].second);
    if (base_link && tip_link)
    {
      // go from tip, up the chain, until we hit the root or we find the base_link
      const LinkModel* lm = tip_link;
      std::vector<const JointModel*> cj;
      while (lm)
      {
        if (lm == base_link)
          break;
        cj.push_back(lm->getParentJointModel());
        lm = lm->getParentJointModel()->getParentLinkModel();
      }
      // if we did not find the base_link, we could have a chain like e.g.,
      // from one end-effector to another end-effector, so the root is in between
      if (lm != base_link)
      {
        // we go up the chain from the base this time, and see where we intersect the other chain
        lm = base_link;
        std::size_t index = 0;
        std::vector<const JointModel*> cj2;
        while (lm)
        {
          for (std::size_t j = 0 ; j < cj.size() ; ++j)
            if (cj[j] == lm->getParentJointModel())
            {
              index = j + 1;
              break;
            }
          if (index > 0)
            break;
          cj2.push_back(lm->getParentJointModel());
          lm = lm->getParentJointModel()->getParentLinkModel();  
        }
        if (index > 0)
        {
          jset.insert(cj.begin(), cj.begin() + index);
          jset.insert(cj2.begin(), cj2.end());
        }
      }
      else
        // if we have a simple chain, just add the joints
        jset.insert(cj.begin(), cj.end());
    }
  }

  // add joints
  for (std::size_t i = 0 ; i < gc.joints_.size() ; ++i)
  {
    const JointModel *j = getJointModel(gc.joints_[i]);
    if (j)
      jset.insert(j);
  }

  // add joints that are parents of included links
  for (std::size_t i = 0 ; i < gc.links_.size() ; ++i)
  {
    const LinkModel *l = getLinkModel(gc.links_[i]);
    if (l)
      jset.insert(l->getParentJointModel());
  }

  // add joints from subgroups
  for (std::size_t i = 0 ; i < gc.subgroups_.size() ; ++i)
  {
    const JointModelGroup *sg = getJointModelGroup(gc.subgroups_[i]);
    if (sg)
    {
      //active joints
      const std::vector<const JointModel*> &js = sg->getJointModels();
      for (std::size_t j = 0 ; j < js.size() ; ++j)
        jset.insert(js[j]);
      
      //fixed joints
      const std::vector<const JointModel*> &fs = sg->getFixedJointModels();
      for (std::size_t j = 0 ; j < fs.size() ; ++j)
        jset.insert(fs[j]);

      //mimic joints
      const std::vector<const JointModel*> &ms = sg->getMimicJointModels();
      for (std::size_t j = 0 ; j < ms.size() ; ++j)
        jset.insert(ms[j]);
    }
  }

  if (jset.empty())
  {
    ROS_WARN_STREAM("Group '" << gc.name_ << "' must have at least one valid joint");
    return false;
  }

  std::vector<const JointModel*> joints;
  for (std::set<const JointModel*>::iterator it = jset.begin() ; it != jset.end() ; ++it)
    joints.push_back(*it);

  std::sort(joints.begin(), joints.end(), &orderJointsByIndex);

  JointModelGroup *jmg = new JointModelGroup(gc.name_, joints, this);
  joint_model_group_map_[gc.name_] = jmg;
  joint_model_group_config_map_[gc.name_] = gc;
  joint_model_group_names_.push_back(gc.name_);
  
  return true;
}

planning_models::KinematicModel::JointModel* planning_models::KinematicModel::buildRecursive(LinkModel *parent, const urdf::Link *link,
                                                                                             const std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > &parent_map,
                                                                                             const std::map<const urdf::Link*, std::vector<const urdf::Link*> > &child_map,
                                                                                             const std::vector<srdf::Model::VirtualJoint> &vjoints)
{      
  std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> >::const_iterator pmi = parent_map.find(link);
  JointModel *joint = (pmi != parent_map.end()) ? constructJointModel(pmi->second.second, link, vjoints) : constructJointModel(NULL, link, vjoints);
  if (joint == NULL)
    return NULL;
  joint_model_map_[joint->name_] = joint;
  joint->tree_index_ = joint_model_vector_.size();
  joint_model_vector_.push_back(joint);
  joint_model_names_vector_.push_back(joint->getName());
  joint->parent_link_model_ = parent;
  joint->child_link_model_ = constructLinkModel(link);
  if (parent == NULL)
    joint->child_link_model_->joint_origin_transform_.setIdentity();
  link_model_map_[joint->child_link_model_->name_] = joint->child_link_model_;
  joint->child_link_model_->tree_index_ = link_model_vector_.size();
  link_model_vector_.push_back(joint->child_link_model_);
  link_model_names_vector_.push_back(link_model_vector_.back()->getName());
  if (joint->child_link_model_->shape_)
  {
    link_models_with_collision_geometry_vector_.push_back(joint->child_link_model_);
    link_model_names_with_collision_geometry_vector_.push_back(link_models_with_collision_geometry_vector_.back()->getName());
  }
  joint->child_link_model_->parent_joint_model_ = joint;
  std::map<const urdf::Link*, std::vector<const urdf::Link*> >::const_iterator cmi = child_map.find(link);
  if (cmi != child_map.end())
    for (unsigned int i = 0 ; i < cmi->second.size() ; ++i)
    {
      JointModel* jm = buildRecursive(joint->child_link_model_, cmi->second[i], parent_map, child_map, vjoints);
      if (jm)
        joint->child_link_model_->child_joint_models_.push_back(jm);
    }
  return joint;
}

planning_models::KinematicModel::JointModel* planning_models::KinematicModel::constructJointModel(const urdf::Joint *urdf_joint, const urdf::Link *child_link,
                                                                                                  const std::vector<srdf::Model::VirtualJoint> &vjoints)
{
  JointModel* result = NULL;

  // must be the root link transform
  if (urdf_joint)
  {
    switch (urdf_joint->type)
    {
    case urdf::Joint::REVOLUTE:
      {
        RevoluteJointModel *j = new RevoluteJointModel(urdf_joint->name);
        if (urdf_joint->safety)
          j->variable_bounds_[0] = std::make_pair(urdf_joint->safety->soft_lower_limit, urdf_joint->safety->soft_upper_limit);
        else
        {
          if (urdf_joint->limits)
            j->variable_bounds_[0] = std::make_pair(urdf_joint->limits->lower, urdf_joint->limits->upper);
          else
            j->variable_bounds_[0] = std::make_pair(0.0, 0.0);
        }
        if (urdf_joint->limits)
          j->max_velocity_ = fabs(urdf_joint->limits->velocity);
        j->continuous_ = false;
        j->axis_ = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::CONTINUOUS:
      {
        RevoluteJointModel *j = new RevoluteJointModel(urdf_joint->name);
        j->continuous_ = true;
        j->variable_bounds_[0] = std::make_pair(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
        if (urdf_joint->limits)
          j->max_velocity_ = fabs(urdf_joint->limits->velocity);
        j->axis_ = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::PRISMATIC:
      {
        PrismaticJointModel *j = new PrismaticJointModel(urdf_joint->name);
        if(urdf_joint->safety)
          j->variable_bounds_[0] = std::make_pair(urdf_joint->safety->soft_lower_limit, urdf_joint->safety->soft_upper_limit);
        else
        {
          if (urdf_joint->limits)
            j->variable_bounds_[0] = std::make_pair(urdf_joint->limits->lower, urdf_joint->limits->upper);  
          else
            j->variable_bounds_[0] = std::make_pair(0.0, 0.0);
        }
        if (urdf_joint->limits)
          j->max_velocity_ = fabs(urdf_joint->limits->velocity);
        j->axis_ = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::FLOATING:
      result = new FloatingJointModel(urdf_joint->name);
      break;
    case urdf::Joint::PLANAR:
      result = new PlanarJointModel(urdf_joint->name);
      break;
    case urdf::Joint::FIXED:
      result = new FixedJointModel(urdf_joint->name);
      break;
    default:
      ROS_ERROR("Unknown joint type: %d", (int)urdf_joint->type);
      break;
    }
  }
  else
  {
    for (std::size_t i = 0 ; i < vjoints.size() ; ++i)
      if (vjoints[i].child_link_ == child_link->name)
      {
        if (vjoints[i].type_ == "fixed")
          result = new FixedJointModel(vjoints[i].name_);
        else if (vjoints[i].type_ == "planar")
          result = new PlanarJointModel(vjoints[i].name_);
        else if (vjoints[i].type_ == "floating")
          result = new FloatingJointModel(vjoints[i].name_);
        if (result)
        {
          model_frame_ = vjoints[i].parent_frame_;
          break;
        }
      }
    if (!result)
    {
      ROS_WARN("No root joint specified. Assuming fixed joint");
      result = new FixedJointModel("ASSUMED_FIXED_ROOT_JOINT");
    }
  }

  if (result)
    for (std::size_t i = 0 ; i < result->variable_names_.size() ; ++i)
      result->variable_index_[result->variable_names_[i]] = i;

  return result;
}

namespace planning_models
{
static inline Eigen::Affine3d urdfPose2Affine3d(const urdf::Pose &pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Affine3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z)*q.toRotationMatrix());
  return af;
}
}

planning_models::KinematicModel::LinkModel* planning_models::KinematicModel::constructLinkModel(const urdf::Link *urdf_link)
{
  ROS_ASSERT(urdf_link);

  LinkModel *result = new LinkModel();
  result->name_ = urdf_link->name;

  if (urdf_link->collision && urdf_link->collision->geometry)
  {
    result->collision_origin_transform_ = urdfPose2Affine3d(urdf_link->collision->origin);
    result->shape_ = constructShape(urdf_link->collision->geometry.get(), result->filename_);
  }
  else if (urdf_link->visual && urdf_link->visual->geometry)
  {
    result->collision_origin_transform_ = urdfPose2Affine3d(urdf_link->visual->origin);
    result->shape_ = constructShape(urdf_link->visual->geometry.get(), result->filename_);
  }
  else
  {
    result->collision_origin_transform_.setIdentity();
    result->shape_.reset();
  }

  if(urdf_link->visual && urdf_link->visual->geometry)
  {
    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(urdf_link->visual->geometry.get());
    if (mesh && !mesh->filename.empty())
      result->visual_filename_ = mesh->filename;
  }
  if (urdf_link->parent_joint.get()) /// \todo this is an issue when we reparent the model
    result->joint_origin_transform_ = urdfPose2Affine3d(urdf_link->parent_joint->parent_to_joint_origin_transform);
  else
    result->joint_origin_transform_.setIdentity();
  return result;
}

shapes::ShapePtr planning_models::KinematicModel::constructShape(const urdf::Geometry *geom, std::string& filename)
{
  ROS_ASSERT(geom);

  shapes::Shape *result = NULL;
  switch (geom->type)
  {
  case urdf::Geometry::SPHERE:
    result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
    break;
  case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
  case urdf::Geometry::CYLINDER:
    result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
                                  dynamic_cast<const urdf::Cylinder*>(geom)->length);
    break;
  case urdf::Geometry::MESH:
    {
      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        result = shapes::createMeshFromFilename(mesh->filename, scale);
        filename = mesh->filename;
      }
    }
    break;
  default:
    ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
    break;
  }

  return shapes::ShapePtr(result);
}

const planning_models::KinematicModel::JointModel* planning_models::KinematicModel::getRoot(void) const
{
  return root_joint_;
}

bool planning_models::KinematicModel::hasJointModel(const std::string &name) const
{
  return joint_model_map_.find(name) != joint_model_map_.end();
}

bool planning_models::KinematicModel::hasLinkModel(const std::string &name) const
{
  return link_model_map_.find(name) != link_model_map_.end();
}

const planning_models::KinematicModel::JointModel* planning_models::KinematicModel::getJointModel(const std::string &name) const
{
  std::map<std::string, JointModel*>::const_iterator it = joint_model_map_.find(name);
  if (it == joint_model_map_.end())
  {
    ROS_ERROR("Joint '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return NULL;
  }
  else
    return it->second;
}

const planning_models::KinematicModel::LinkModel* planning_models::KinematicModel::getLinkModel(const std::string &name) const
{
  std::map<std::string, LinkModel*>::const_iterator it = link_model_map_.find(name);
  if (it == link_model_map_.end())
  {
    ROS_ERROR("Link '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

const std::vector<std::string>& planning_models::KinematicModel::getJointModelGroupNames(void) const
{
  return joint_model_group_names_;
}

void planning_models::KinematicModel::getChildLinkModels(const KinematicModel::LinkModel *parent, std::vector<const KinematicModel::LinkModel*> &links) const
{
  links.clear();
  links.push_back(parent);
  std::queue<const LinkModel*> q;
  std::set<const LinkModel*> seen;
  q.push(parent);
  while (!q.empty())
  {
    const LinkModel* t = q.front();
    q.pop();
    if (seen.insert(t).second)
      for (std::size_t i = 0 ; i < t->child_joint_models_.size() ; ++i)
      {
        links.push_back(t->child_joint_models_[i]->child_link_model_);
        q.push(t->child_joint_models_[i]->child_link_model_);
        for (std::size_t j = 0 ; j < t->child_joint_models_[i]->mimic_requests_.size() ; ++j)
        {
          links.push_back(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
          q.push(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
        }
      }
  }
}

void planning_models::KinematicModel::getChildLinkModels(const KinematicModel::JointModel *parent, std::vector<const KinematicModel::LinkModel*> &links) const
{
  getChildLinkModels(parent->child_link_model_, links);
}

void planning_models::KinematicModel::getChildJointModels(const KinematicModel::LinkModel *parent, std::vector<const KinematicModel::JointModel*> &joints) const
{
  joints.clear();
  std::queue<const LinkModel*> q;
  std::set<const LinkModel*> seen;
  q.push(parent);

  while (!q.empty())
  {
    const LinkModel* t = q.front();
    q.pop();
    if (seen.insert(t).second)
      for (unsigned int i = 0 ; i < t->child_joint_models_.size() ; ++i)
      {
        joints.push_back(t->child_joint_models_[i]);
        q.push(t->child_joint_models_[i]->child_link_model_);
        for (std::size_t j = 0 ; j < t->child_joint_models_[i]->mimic_requests_.size() ; ++j)
        {
          joints.push_back(t->child_joint_models_[i]->mimic_requests_[j]);
          q.push(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
        }
      }
  }
}

void planning_models::KinematicModel::getChildJointModels(const KinematicModel::JointModel *parent, std::vector<const KinematicModel::JointModel*> &joints) const
{
  getChildJointModels(parent->child_link_model_, joints);
  joints.insert(joints.begin(), parent);
}

std::vector<std::string> planning_models::KinematicModel::getChildLinkModelNames(const KinematicModel::LinkModel *parent) const
{
  std::vector<const LinkModel*> links;
  getChildLinkModels(parent, links);
  std::vector<std::string> ret_vec(links.size());
  for (std::size_t i = 0; i < links.size(); ++i)
    ret_vec[i] = links[i]->getName();
  return ret_vec;
}

std::vector<std::string> planning_models::KinematicModel::getChildLinkModelNames(const KinematicModel::JointModel *parent) const
{
  std::vector<const LinkModel*> links;
  getChildLinkModels(parent, links);
  std::vector<std::string> ret_vec(links.size());
  for(unsigned int i = 0; i < links.size(); ++i)
    ret_vec[i] = links[i]->getName();
  return ret_vec;
}

std::vector<std::string> planning_models::KinematicModel::getChildJointModelNames(const KinematicModel::LinkModel *parent) const
{
  std::vector<const KinematicModel::JointModel*> joints;
  getChildJointModels(parent, joints);
  std::vector<std::string> ret_vec(joints.size());
  for(unsigned int i = 0 ; i < joints.size() ; ++i)
    ret_vec[i] = joints[i]->getName();
  return ret_vec;
}

std::vector<std::string> planning_models::KinematicModel::getChildJointModelNames(const KinematicModel::JointModel *parent) const
{
  std::vector<const KinematicModel::JointModel*> joints;
  getChildJointModels(parent, joints);
  std::vector<std::string> ret_vec(joints.size());
  for(unsigned int i = 0 ; i < joints.size(); ++i)
    ret_vec[i] = joints[i]->getName();
  return ret_vec;
}


void planning_models::KinematicModel::getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_vector_[i]->mimic_ == NULL)
      joint_model_vector_[i]->getRandomValues(rng, values);
}

void planning_models::KinematicModel::getDefaultValues(std::vector<double> &values) const
{
    for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
        if (joint_model_vector_[i]->mimic_ == NULL)
            joint_model_vector_[i]->getDefaultValues(values);
}

/* ------------------------ JointModel ------------------------ */

planning_models::KinematicModel::JointModel::JointModel(const std::string& name) :
  name_(name), type_(UNKNOWN), max_velocity_(0.0), parent_link_model_(NULL), child_link_model_(NULL),
  mimic_(NULL), mimic_factor_(1.0), mimic_offset_(0.0), tree_index_(-1)
{
}

planning_models::KinematicModel::JointModel::~JointModel(void)
{
}

bool planning_models::KinematicModel::JointModel::getVariableBounds(const std::string& variable, std::pair<double, double>& bounds) const
{
  std::map<std::string, unsigned int>::const_iterator it = variable_index_.find(variable);
  if (it == variable_index_.end())
  {
    ROS_WARN_STREAM("Could not find variable '" << variable << "' to get bounds for within joint '" << name_ << "'");
    return false;
  }
  bounds = variable_bounds_[it->second];
  return true;
}

void planning_models::KinematicModel::JointModel::getDefaultValues(std::map<std::string, double> &values) const
{
  std::vector<double> defv;
  defv.reserve(variable_names_.size());
  getDefaultValues(defv);
  for (std::size_t i = 0 ; i < variable_names_.size() ; ++i)
    values[variable_names_[i]] = defv[i];
}

void planning_models::KinematicModel::JointModel::getDefaultValues(std::vector<double> &values) const
{
  for (std::vector<std::pair<double, double> >::const_iterator it = variable_bounds_.begin() ; it != variable_bounds_.end() ; ++it)
  {
    // if zero is a valid value
    if (it->first <= 0.0 && it->second >= 0.0)
      values.push_back(0.0);
    else
      values.push_back((it->first + it->second)/2.0);
  }
}

void planning_models::KinematicModel::JointModel::getRandomValues(random_numbers::RandomNumberGenerator &rng, std::map<std::string, double> &values) const
{
  std::vector<double> rv;
  rv.reserve(variable_names_.size());
  getRandomValues(rng, rv);
  for (std::size_t i = 0 ; i < variable_names_.size() ; ++i)
    values[variable_names_[i]] = rv[i];
}

void planning_models::KinematicModel::JointModel::getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const
{
  for (std::vector<std::pair<double, double> >::const_iterator it = variable_bounds_.begin() ; it != variable_bounds_.end() ; ++it)
    if (it->first > -std::numeric_limits<double>::max() && it->second < std::numeric_limits<double>::max())
      values.push_back(rng.uniformReal(it->first, it->second));
    else
      values.push_back(0.0);
}

bool planning_models::KinematicModel::JointModel::isVariableWithinBounds(const std::string& variable, double value) const
{
  std::pair<double, double> bounds;
  if (!getVariableBounds(variable, bounds))
    return false;
  if (value < bounds.first || value > bounds.second)
    return false;
  return true;
}

bool planning_models::KinematicModel::JointModel::satisfiesBounds(const std::vector<double> &values) const
{
  for (std::size_t i = 0 ; i < values.size() ; ++i)
    if (!isVariableWithinBounds(variable_names_[i], values[i]))
      return false;
  return true;
}

void planning_models::KinematicModel::JointModel::enforceBounds(std::vector<double> &values) const
{
  for (std::size_t i = 0 ; i < variable_bounds_.size() ; ++i)
  {
    const std::pair<double, double> &bounds = variable_bounds_[i];
    if (values[i] < bounds.first)
      values[i] = bounds.first;
    else
      if (values[i] > bounds.second)
        values[i] = bounds.second;
  }
}

std::vector<moveit_msgs::JointLimits> planning_models::KinematicModel::JointModel::getJointLimits(void) const 
{
  std::vector<moveit_msgs::JointLimits> ret_vec;
  for (std::size_t i = 0; i < variable_names_.size(); ++i)
  {
    moveit_msgs::JointLimits lim;
    lim.joint_name = variable_names_[i];
    lim.has_position_limits = true;
    lim.min_position = variable_bounds_[i].first;
    lim.min_position = variable_bounds_[i].second;
    if (max_velocity_ != 0.0)
      lim.has_velocity_limits = true;
    else
      lim.has_velocity_limits = false;
    lim.max_velocity = max_velocity_;
    lim.has_acceleration_limits = false;
    lim.max_acceleration = 0.0;
    ret_vec.push_back(lim);
  }
  return ret_vec;
}

planning_models::KinematicModel::FixedJointModel::FixedJointModel(const std::string& name) : JointModel(name)
{
  type_ = FIXED;
}

void planning_models::KinematicModel::FixedJointModel::computeTransform(const std::vector<double>& /* joint_values */, Eigen::Affine3d &transf) const
{
  transf.setIdentity();
}

void planning_models::KinematicModel::FixedJointModel::updateTransform(const std::vector<double>& /* joint_values */, Eigen::Affine3d &transf) const
{
}

void planning_models::KinematicModel::FixedJointModel::computeJointStateValues(const Eigen::Affine3d& /* transform */, std::vector<double>& joint_values) const
{
  joint_values.clear();
}

planning_models::KinematicModel::PlanarJointModel::PlanarJointModel(const std::string& name) : JointModel(name)
{
  local_names_.push_back("x");
  local_names_.push_back("y");
  local_names_.push_back("theta");
  for (int i = 0 ; i < 3 ; ++i)
    variable_names_.push_back(name_ + "/" + local_names_[i]);
  variable_bounds_.resize(3);
  variable_bounds_[0] = std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  variable_bounds_[1] = std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  variable_bounds_[2] = std::make_pair(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
  type_ = PLANAR;
}

bool planning_models::KinematicModel::PlanarJointModel::isVariableWithinBounds(const std::string& variable, double value) const
{
  std::map<std::string, unsigned int>::const_iterator it = variable_index_.find(variable);
  if (it != variable_index_.end() && it->second == 2)
    // if we are inquiring about the yaw of the robot, that is essentially a continuous joint, so there are no bounds (angle wraps)
    return true;
  else
    return JointModel::isVariableWithinBounds(variable, value);
}

void planning_models::KinematicModel::PlanarJointModel::normalizeRotation(std::vector<double> &values) const
{
  double &v = values[2];
  v = fmod(v, 2.0 * boost::math::constants::pi<double>());
  if (v < -boost::math::constants::pi<double>())
    v += 2.0 * boost::math::constants::pi<double>();
  else
    if (v > boost::math::constants::pi<double>())
      v -= 2.0 * boost::math::constants::pi<double>();
}

void planning_models::KinematicModel::PlanarJointModel::enforceBounds(std::vector<double> &values) const
{
  normalizeRotation(values);
  JointModel::enforceBounds(values);
}

void planning_models::KinematicModel::PlanarJointModel::computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  updateTransform(joint_values, transf);
}

void planning_models::KinematicModel::PlanarJointModel::updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf = Eigen::Affine3d(Eigen::Translation3d(joint_values[0], joint_values[1], 0.0)*Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitZ()));
}

void planning_models::KinematicModel::PlanarJointModel::computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const
{
  joint_values.resize(3);
  joint_values[0] = transf.translation().x();
  joint_values[1] = transf.translation().y();
  
  Eigen::Quaterniond q(transf.rotation());
  //taken from Bullet
  double s_squared = 1.0-(q.w()*q.w());
  if (s_squared < 10.0 * std::numeric_limits<double>::epsilon())
    joint_values[2] = 0.0;
  else
  {
    double s = 1.0/sqrt(s_squared);
    joint_values[2] = (acos(q.w())*2.0f)*(q.z()*s);
  }
}

planning_models::KinematicModel::FloatingJointModel::FloatingJointModel(const std::string& name) : JointModel(name)
{
  local_names_.push_back("trans_x");
  local_names_.push_back("trans_y");
  local_names_.push_back("trans_z");
  local_names_.push_back("rot_x");
  local_names_.push_back("rot_y");
  local_names_.push_back("rot_z");
  local_names_.push_back("rot_w");
  for (int i = 0 ; i < 7 ; ++i)
    variable_names_.push_back(name_ + "/" + local_names_[i]);
  variable_bounds_.resize(7);
  variable_bounds_[0] = std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  variable_bounds_[1] = std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  variable_bounds_[2] = std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  variable_bounds_[3] = std::make_pair(-1.0, 1.0);
  variable_bounds_[4] = std::make_pair(-1.0, 1.0);
  variable_bounds_[5] = std::make_pair(-1.0, 1.0);
  variable_bounds_[6] = std::make_pair(-1.0, 1.0);
  type_ = FLOATING;
}

void planning_models::KinematicModel::FloatingJointModel::normalizeRotation(std::vector<double> &values) const
{ 
  // normalize the quaternion if we need to
  double normSqr = values[3] * values[3] + values[4] * values[4] + values[5] * values[5] + values[6] * values[6];
  if (fabs(normSqr - 1.0) > std::numeric_limits<double>::epsilon() * 100.0)
  {
    double norm = sqrt(normSqr);
    if (norm < std::numeric_limits<double>::epsilon() * 100.0)
    {
      ROS_WARN("Quaternion is not normalized in KinematicState representation. Setting to identity");
      values[3] = 0.0;
      values[4] = 0.0;
      values[5] = 0.0;
      values[6] = 1.0;
    }
    else
    {
      values[3] /= norm;
      values[4] /= norm;
      values[5] /= norm;
      values[6] /= norm;
    }
  }
}

void planning_models::KinematicModel::FloatingJointModel::enforceBounds(std::vector<double> &values) const
{
  normalizeRotation(values);
  JointModel::enforceBounds(values);
}

void planning_models::KinematicModel::FloatingJointModel::computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  updateTransform(joint_values, transf);
}

void planning_models::KinematicModel::FloatingJointModel::updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf = Eigen::Affine3d(Eigen::Translation3d(joint_values[0], joint_values[1], joint_values[2])
                           *Eigen::Quaterniond(joint_values[6],joint_values[3], joint_values[4], joint_values[5]).toRotationMatrix());
}

void planning_models::KinematicModel::FloatingJointModel::computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const
{
  joint_values.resize(7);
  joint_values[0] = transf.translation().x();
  joint_values[1] = transf.translation().y();
  joint_values[2] = transf.translation().z();
  Eigen::Quaterniond q(transf.rotation());
  joint_values[3] = q.x();
  joint_values[4] = q.y();
  joint_values[5] = q.z();
  joint_values[6] = q.w();
}

void planning_models::KinematicModel::FloatingJointModel::getDefaultValues(std::vector<double>& values) const
{
  JointModel::getDefaultValues(values);
  std::size_t s = values.size();
  values[s - 4] = 0.0;
  values[s - 3] = 0.0;
  values[s - 2] = 0.0;
  values[s - 1] = 1.0;
}

void planning_models::KinematicModel::FloatingJointModel::getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const
{
  std::size_t s = values.size();
  values.resize(s + 7);
  values[s] = rng.uniformReal(variable_bounds_[0].first, variable_bounds_[0].second);
  values[s + 1] = rng.uniformReal(variable_bounds_[1].first, variable_bounds_[1].second);
  values[s + 2] = rng.uniformReal(variable_bounds_[2].first, variable_bounds_[2].second);
  double q[4]; rng.quaternion(q);
  values[s + 3] = q[0];
  values[s + 4] = q[1];
  values[s + 5] = q[2];
  values[s + 6] = q[3];
}

planning_models::KinematicModel::PrismaticJointModel::PrismaticJointModel(const std::string& name) : JointModel(name), axis_(0.0, 0.0, 0.0)
{
  variable_bounds_.push_back(std::make_pair(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max()));
  variable_names_.push_back(name_);
  type_ = PRISMATIC;
}

void planning_models::KinematicModel::PrismaticJointModel::computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf.setIdentity();
  updateTransform(joint_values, transf);
}

void planning_models::KinematicModel::PrismaticJointModel::updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf.translation() = Eigen::Vector3d(axis_ * joint_values[0]);
}

void planning_models::KinematicModel::PrismaticJointModel::computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const
{
  joint_values.resize(1);
  joint_values[0] = transf.translation().dot(axis_);
}

planning_models::KinematicModel::RevoluteJointModel::RevoluteJointModel(const std::string& name) : JointModel(name),
                                                                                                   axis_(0.0, 0.0, 0.0), continuous_(false)
{
  variable_bounds_.push_back(std::make_pair(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>()));
  variable_names_.push_back(name_);
  type_ = REVOLUTE;
}

bool planning_models::KinematicModel::RevoluteJointModel::isVariableWithinBounds(const std::string& variable, double value) const
{
  if (continuous_)
    return true;
  else
    return JointModel::isVariableWithinBounds(variable, value);
}

void planning_models::KinematicModel::RevoluteJointModel::enforceBounds(std::vector<double> &values) const
{
  if (continuous_)
  {
    double &v = values[0];
    v = fmod(v, 2.0 * boost::math::constants::pi<double>());
    if (v < -boost::math::constants::pi<double>())
      v += 2.0 * boost::math::constants::pi<double>();
    else
      if (v > boost::math::constants::pi<double>())
        v -= 2.0 * boost::math::constants::pi<double>();
  }
  else
    JointModel::enforceBounds(values);
}

std::vector<moveit_msgs::JointLimits> planning_models::KinematicModel::RevoluteJointModel::getJointLimits(void) const
{
  std::vector<moveit_msgs::JointLimits> ret_vec = JointModel::getJointLimits();
  if (continuous_)
    ret_vec[0].has_position_limits = false;
  return ret_vec;
}

void planning_models::KinematicModel::RevoluteJointModel::computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  updateTransform(joint_values, transf);
}

void planning_models::KinematicModel::RevoluteJointModel::updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf = Eigen::Affine3d(Eigen::AngleAxisd(joint_values[0], axis_)); 
}

void planning_models::KinematicModel::RevoluteJointModel::computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const
{
  joint_values.resize(1);
  Eigen::Quaterniond q(transf.rotation());
  q.normalize();
  joint_values[0] = acos(q.w())*2.0f;
}

/* ------------------------ LinkModel ------------------------ */

planning_models::KinematicModel::LinkModel::LinkModel(void) : parent_joint_model_(NULL), tree_index_(-1)
{
  joint_origin_transform_.setIdentity();
  collision_origin_transform_.setIdentity();
}

planning_models::KinematicModel::LinkModel::~LinkModel(void)
{
}

/* ------------------------ JointModelGroup ------------------------ */
planning_models::KinematicModel::JointModelGroup::JointModelGroup(const std::string& group_name,
                                                                  const std::vector<const JointModel*> &group_joints,
                                                                  const KinematicModel* parent_model) :
  parent_model_(parent_model), name_(group_name)
{
  variable_count_ = 0;
  for (std::size_t i = 0 ; i < group_joints.size() ; ++i)
  {
    joint_model_map_[group_joints[i]->getName()] = group_joints[i];
    unsigned int vc = group_joints[i]->getVariableCount();
    if (vc > 0)
    {
      if (group_joints[i]->getMimic() == NULL)
      {
        joint_model_vector_.push_back(group_joints[i]);
        joint_model_name_vector_.push_back(group_joints[i]->getName());
        variable_count_ += vc;
      }
      else
        mimic_joints_.push_back(group_joints[i]);
    }
    else
      fixed_joints_.push_back(group_joints[i]);
  }

  //now we need to find all the set of joints within this group
  //that root distinct subtrees
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    bool found = false;
    const JointModel *joint = joint_model_vector_[i];
    //if we find that an ancestor is also in the group, then the joint is not a root
    while (joint->getParentLinkModel() != NULL)
    {
      joint = joint->getParentLinkModel()->getParentJointModel();
      if (hasJointModel(joint->name_) && joint->getVariableCount() > 0 && joint->getMimic() == NULL)
      {
        found = true;
        break;
      }
    }
    if (!found)
      joint_roots_.push_back(joint_model_vector_[i]);
  }

  // compute joint_variable_index_map_
  unsigned int vector_index_counter = 0;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const std::vector<std::string>& name_order = joint_model_vector_[i]->getVariableNames();
    for (std::size_t j = 0; j < name_order.size(); ++j)
    {
      joint_variables_index_map_[name_order[j]] = vector_index_counter + j;
      active_dof_names_.push_back(name_order[j]);
    }
    joint_variables_index_map_[joint_model_vector_[i]->getName()] = vector_index_counter;
    vector_index_counter += name_order.size();
  }

  for (std::size_t i = 0 ; i < mimic_joints_.size() ; ++i)
  {
    const std::vector<std::string>& name_order = mimic_joints_[i]->getVariableNames();
    const std::vector<std::string>& mim_name_order = mimic_joints_[i]->mimic_->getVariableNames();
    for (std::size_t j = 0; j < name_order.size(); ++j)
      joint_variables_index_map_[name_order[j]] = joint_variables_index_map_[mim_name_order[j]];
    joint_variables_index_map_[mimic_joints_[i]->getName()] = joint_variables_index_map_[mimic_joints_[i]->mimic_->getName()];
  }

  // now we need to make another pass for group links (we include the fixed joints here)
  std::set<const LinkModel*> group_links_set;
  for (std::size_t i = 0 ; i < group_joints.size() ; ++i)
    group_links_set.insert(group_joints[i]->getChildLinkModel());
  
  for (std::set<const LinkModel*>::iterator it = group_links_set.begin(); it != group_links_set.end(); ++it)
    link_model_vector_.push_back(*it);
  
  std::sort(link_model_vector_.begin(), link_model_vector_.end(), &orderLinksByIndex);
  for (std::size_t i = 0 ; i < link_model_vector_.size() ; ++i)
    link_model_name_vector_.push_back(link_model_vector_[i]->getName());

  // compute updated links
  std::set<const LinkModel*> u_links;
  for (std::size_t i = 0 ; i < joint_roots_.size() ; ++i)
  {
    std::vector<const LinkModel*> links;
    parent_model->getChildLinkModels(joint_roots_[i], links);
    u_links.insert(links.begin(), links.end());
  }
  for (std::set<const LinkModel*>::iterator it = u_links.begin(); it != u_links.end(); ++it)
    updated_link_model_vector_.push_back(*it);
  std::sort(updated_link_model_vector_.begin(), updated_link_model_vector_.end(), &orderLinksByIndex);
  for (std::size_t i = 0; i < updated_link_model_vector_.size(); i++)
    updated_link_model_name_vector_.push_back(updated_link_model_vector_[i]->getName());
}

planning_models::KinematicModel::JointModelGroup::~JointModelGroup(void)
{
}

bool planning_models::KinematicModel::JointModelGroup::isSubgroup(const std::string& group) const
{
  for (std::size_t i = 0; i < subgroup_names_.size(); ++i)
    if (group == subgroup_names_[i])
      return true;
  return false;
}

bool planning_models::KinematicModel::JointModelGroup::isDisjointSubgroup(const std::string& group) const
{
  for (std::size_t i = 0; i < disjoint_subgroup_names_.size(); ++i)
    if (group == disjoint_subgroup_names_[i])
      return true;
  return false;
}

bool planning_models::KinematicModel::JointModelGroup::hasJointModel(const std::string &joint) const
{
  return joint_model_map_.find(joint) != joint_model_map_.end();
}

bool planning_models::KinematicModel::JointModelGroup::hasLinkModel(const std::string &link) const
{
  return std::find(link_model_name_vector_.begin(), link_model_name_vector_.end(), link) != link_model_name_vector_.end();
}

const planning_models::KinematicModel::JointModel* planning_models::KinematicModel::JointModelGroup::getJointModel(const std::string &name) const
{
  std::map<std::string, const JointModel*>::const_iterator it = joint_model_map_.find(name);
  if (it == joint_model_map_.end())
  {
    ROS_ERROR("Joint '%s' not found in group '%s'", name.c_str(), name_.c_str());
    return NULL;
  }
  else
    return it->second;
}

void planning_models::KinematicModel::JointModelGroup::getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getRandomValues(rng, values);
}

bool planning_models::KinematicModel::JointModelGroup::getDefaultValues(const std::string &name, std::map<std::string, double> &values) const
{
  std::map<std::string, std::map<std::string, double> >::const_iterator it = default_states_.find(name);
  if (it == default_states_.end())
    return false;
  values = it->second;
  return true;
}

std::vector<moveit_msgs::JointLimits> planning_models::KinematicModel::JointModelGroup::getJointLimits(void) const
{
  std::vector<moveit_msgs::JointLimits> ret_vec;
  for(unsigned int i = 0; i < joint_model_vector_.size(); i++)
  {
    std::vector<moveit_msgs::JointLimits> jvec = joint_model_vector_[i]->getJointLimits();
    ret_vec.insert(ret_vec.end(), jvec.begin(), jvec.end());
  }
  return ret_vec;
}

void planning_models::KinematicModel::printModelInfo(std::ostream &out) const
{
  out << "Model " << model_name_ << " in frame " << model_frame_ << ", of dimension " << getVariableCount() << std::endl;

  std::ios_base::fmtflags old_flags = out.flags();
  out.setf(std::ios::fixed, std::ios::floatfield);
  std::streamsize old_prec = out.precision();
  out.precision(5);
  out << "Joint values bounds: " << std::endl;
  for (unsigned int i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const std::vector<std::string> &vn = joint_model_vector_[i]->getVariableNames();
    for (std::vector<std::string>::const_iterator it = vn.begin() ; it != vn.end() ; ++it)
    {
      out << "   " << *it << " [";
      std::pair<double, double> b;
      joint_model_vector_[i]->getVariableBounds(*it, b);
      if (b.first <= -std::numeric_limits<double>::max())
        out << "DBL_MIN";
      else
        out << b.first;
      out << ", ";
      if (b.second >= std::numeric_limits<double>::max())
        out << "DBL_MAX";
      else
        out << b.second;
      out << "]";
      if (joint_model_vector_[i]->mimic_)
        out << " *";
      out << std::endl;
    }
  }
  out << std::endl;
  out.precision(old_prec);
  out.flags(old_flags);

  out << "Available groups: " << std::endl;
  for (std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
  {
    out << "   " << it->first << " (of dimension " << it->second->getVariableCount() << "):" << std::endl;
    out << "     joints:" << std::endl;
    const std::vector<std::string> &jnt = it->second->getJointModelNames();
    for (std::size_t k = 0 ; k < jnt.size() ; ++k)
      out << "      " << jnt[k] << std::endl;
    out << "     links:" << std::endl;
    const std::vector<std::string> &lnk = it->second->getLinkModelNames();
    for (std::size_t k = 0 ; k < lnk.size() ; ++k)
      out << "      " << lnk[k] << std::endl;
    out << "     roots:" << std::endl;
    const std::vector<const JointModel*> &jr = it->second->getJointRoots();
    for (std::size_t k = 0 ; k < jr.size() ; ++k)
      out << "      " << jr[k]->getName() << std::endl;

  }
}
