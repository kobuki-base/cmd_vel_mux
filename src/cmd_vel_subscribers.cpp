/**
 * @file /src/cmd_vel_subscribers.cpp
 *
 * @brief  Subscriber handlers for the yocs_cmd_vel_mux
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include <memory>
#include <vector>

#include "cmd_vel_mux/cmd_vel_subscribers.hpp"
#include "cmd_vel_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void CmdVelSubscribers::CmdVelSub::operator << (const YAML::Node& node)
{
  // Fill attributes with a YAML node content
  double new_timeout;
  std::string new_topic;
  node["name"]     >> name_;
  node["topic"]    >> new_topic;
  node["timeout"]  >> new_timeout;
  node["priority"] >> priority_;
#ifdef HAVE_NEW_YAMLCPP
  if (node["short_desc"]) {
#else
  if (node.FindValue("short_desc") != nullptr) {
#endif
    node["short_desc"] >> short_desc_;
  }

  if (new_topic != topic_)
  {
    // Shutdown the topic if the name has changed so it gets recreated on configuration reload
    // In the case of new subscribers, topic is empty and shutdown has just no effect
    topic_ = new_topic;
    sub_.shutdown();
  }

  if (new_timeout != timeout_)
  {
    // Change timer period if the timeout changed
    timeout_ = new_timeout;
    timer_.setPeriod(ros::Duration(timeout_));
  }
}

unsigned int CmdVelSubscribers::CmdVelSub::getPriority() const
{
  return priority_;
}

std::vector<std::shared_ptr<CmdVelSubscribers::CmdVelSub>>::size_type CmdVelSubscribers::size()
{
  return list_.size();
}

std::shared_ptr<CmdVelSubscribers::CmdVelSub>& CmdVelSubscribers::operator[](unsigned int idx)
{
  return list_[idx];
}

void CmdVelSubscribers::configure(const YAML::Node& node)
{
  try
  {
    if (node.size() == 0)
    {
      throw EmptyCfgException("Configuration is empty");
    }

    std::vector<std::shared_ptr<CmdVelSub>> new_list(node.size());
    for (unsigned int i = 0; i < node.size(); i++)
    {
      // Parse entries on YAML
      std::string new_sub_name = node[i]["name"].Scalar();
      auto old_sub = std::find_if(list_.begin(), list_.end(),
                                  [&new_sub_name](const std::shared_ptr<CmdVelSub>& sub)
                                                  {return sub->name_ == new_sub_name;});
      if (old_sub != list_.end())
      {
        // For names already in the subscribers list, retain current object so we don't re-subscribe to the topic
        new_list[i] = *old_sub;
      }
      else
      {
        new_list[i] = std::make_shared<CmdVelSub>();
      }
      // update existing or new object with the new configuration
      *new_list[i] << node[i];
    }

    list_ = new_list;
  }
  catch (const YAML::ParserException& e)
  {
    throw YamlException(e.what());
  }
  catch (const YAML::RepresentationException& e)
  {
    throw YamlException(e.what());
  }
}


} // namespace cmd_vel_mux
