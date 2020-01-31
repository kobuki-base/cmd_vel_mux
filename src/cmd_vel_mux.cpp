/**
 * @file /src/cmd_vel_mux_nodelet.cpp
 *
 * @brief  Implementation for the command velocity multiplexer
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>

#include "cmd_vel_mux/cmd_vel_mux.hpp"
#include "cmd_vel_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{

/*********************
 ** Private Classes
 **********************/
class CmdVelMux::CmdVelFunctor
{
private:
  unsigned int idx;
  CmdVelMux* node;

public:
  CmdVelFunctor(unsigned int idx, CmdVelMux* node) :
      idx(idx), node(node)
  {
  }

  void operator()(const geometry_msgs::Twist::ConstPtr& msg)
  {
    node->cmdVelCallback(msg, idx);
  }
};

class CmdVelMux::TimerFunctor
{
private:
  unsigned int idx;
  CmdVelMux* node;

public:
  TimerFunctor(unsigned int idx, CmdVelMux* node) :
      idx(idx), node(node)
  {
  }

  void operator()(const ros::TimerEvent& event)
  {
    node->timerCallback(event, idx);
  }
};

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

CmdVelMux::CmdVelMux()
{
  cmd_vel_subs_.allowed_ = VACANT;
  dynamic_reconfigure_server_ = nullptr;
}

CmdVelMux::~CmdVelMux()
{
  if (dynamic_reconfigure_server_ != nullptr)
  {
    delete dynamic_reconfigure_server_;
  }
}

void CmdVelMux::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx)
{
  // Reset general timer
  common_timer_.stop();
  common_timer_.start();

  // Reset timer for this source
  cmd_vel_subs_[idx]->timer_.stop();
  cmd_vel_subs_[idx]->timer_.start();

  cmd_vel_subs_[idx]->active_ = true;   // obviously his source is sending commands, so active

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((cmd_vel_subs_.allowed_ == VACANT) ||
      (cmd_vel_subs_.allowed_ == idx)    ||
      (cmd_vel_subs_[idx]->priority_ > cmd_vel_subs_[cmd_vel_subs_.allowed_]->priority_))
  {
    if (cmd_vel_subs_.allowed_ != idx)
    {
      cmd_vel_subs_.allowed_ = idx;

      // Notify the world that a new cmd_vel source took the control
      std_msgs::StringPtr acv_msg(new std_msgs::String);
      acv_msg->data = cmd_vel_subs_[idx]->name_;
      active_subscriber_.publish(acv_msg);
    }

    output_topic_pub_.publish(msg);
  }
}

void CmdVelMux::timerCallback(const ros::TimerEvent& event, unsigned int idx)
{
  if (cmd_vel_subs_.allowed_ == idx || (idx == GLOBAL_TIMER && cmd_vel_subs_.allowed_ != VACANT))
  {
    if (idx == GLOBAL_TIMER)
    {
      // No cmd_vel messages timeout happened for ANYONE, so last active source got stuck without further
      // messages; not a big problem, just dislodge it; but possibly reflect a problem in the controller
      NODELET_WARN("CmdVelMux : No cmd_vel messages from ANY input received in the last %fs", common_timer_period_);
      NODELET_WARN("CmdVelMux : %s dislodged due to general timeout",
                   cmd_vel_subs_[cmd_vel_subs_.allowed_]->name_.c_str());
    }

    // No cmd_vel messages timeout happened to currently active source, so...
    cmd_vel_subs_.allowed_ = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    std_msgs::StringPtr acv_msg(new std_msgs::String);
    acv_msg->data = "idle";
    active_subscriber_.publish(acv_msg);
  }

  if (idx != GLOBAL_TIMER)
  {
    cmd_vel_subs_[idx]->active_ = false;
  }
}

void CmdVelMux::onInit()
{
  ros::NodeHandle &nh = this->getPrivateNodeHandle();

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure_cb_ = boost::bind(&CmdVelMux::reloadConfiguration, this, _1, _2);
  dynamic_reconfigure_server_ = new dynamic_reconfigure::Server<cmd_vel_mux::reloadConfig>(nh);
  dynamic_reconfigure_server_->setCallback(dynamic_reconfigure_cb_);

  active_subscriber_ = nh.advertise <std_msgs::String> ("active", 1, true); // latched topic

  // Notify the world that by now nobody is publishing on cmd_vel yet
  std_msgs::StringPtr active_msg(new std_msgs::String);
  active_msg->data = "idle";
  active_subscriber_.publish(active_msg);

  // could use a call to reloadConfiguration here, but it seems to automatically call it once with defaults anyway.
  NODELET_DEBUG("CmdVelMux : successfully initialized");
}

void CmdVelMux::reloadConfiguration(cmd_vel_mux::reloadConfig &config, uint32_t unused_level)
{
  ros::NodeHandle &pnh = this->getPrivateNodeHandle();

  std::unique_ptr<std::istream> is;

  // Configuration can come directly as a yaml-formatted string or as a file path,
  // but not both, so we give priority to the first option
  if (config.yaml_cfg_data.size() > 0)
  {
    is.reset(new std::istringstream(config.yaml_cfg_data));
  }
  else
  {
    std::string yaml_cfg_file;
    if (config.yaml_cfg_file == "")
    {
      // typically fired on startup, so look for a parameter to set a default
      pnh.getParam("yaml_cfg_file", yaml_cfg_file);
    }
    else
    {
      yaml_cfg_file = config.yaml_cfg_file;
    }

    is.reset(new std::ifstream(yaml_cfg_file.c_str(), std::ifstream::in));
    if (is->good() == false)
    {
      NODELET_ERROR_STREAM("CmdVelMux : configuration file not found [" << yaml_cfg_file << "]");
      return;
    }
  }

  /*********************
  ** Yaml File Parsing
  **********************/

  // probably need to bring the try catches back here
  YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
  doc = YAML::Load(*is);
#else
  YAML::Parser parser(*is);
  parser.GetNextDocument(doc);
#endif

  /*********************
  ** Output Publisher
  **********************/
  std::string output_name("output");
#ifdef HAVE_NEW_YAMLCPP
  if (doc["publisher"])
  {
    doc["publisher"] >> output_name;
  }
#else
  const YAML::Node *node = doc.FindValue("publisher");
  if (node != nullptr)
  {
    *node >> output_name;
  }
#endif

  if (output_topic_name_ != output_name)
  {
    output_topic_name_ = output_name;
    output_topic_pub_ = pnh.advertise<geometry_msgs::Twist>(output_topic_name_, 10);
    NODELET_DEBUG_STREAM("CmdVelMux : subscribe to output topic '" << output_name << "'");
  }
  else
  {
    NODELET_DEBUG_STREAM("CmdVelMux : no need to re-subscribe to output topic '" << output_name << "'");
  }

  /*********************
  ** Input Subscribers
  **********************/
  try
  {
    cmd_vel_subs_.configure(doc["subscribers"]);
  }
  catch (const EmptyCfgException& e)
  {
    NODELET_WARN_STREAM("CmdVelMux : yaml configured zero subscribers, check yaml content");
  }
  catch (const YamlException& e)
  {
    NODELET_ERROR_STREAM("CmdVelMux : yaml parsing problem [" << std::string(e.what()) << "]");
  }

  // (Re)create subscribers whose topic is invalid: new ones and those with changed names
  double longest_timeout = 0.0;
  for (unsigned int i = 0; i < cmd_vel_subs_.size(); i++)
  {
    if (!cmd_vel_subs_[i]->subs_)
    {
      cmd_vel_subs_[i]->subs_ =
          pnh.subscribe<geometry_msgs::Twist>(cmd_vel_subs_[i]->topic_, 10, CmdVelFunctor(i, this));
      NODELET_DEBUG("CmdVelMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
                    cmd_vel_subs_[i]->name_.c_str(), cmd_vel_subs_[i]->topic_.c_str(),
                    cmd_vel_subs_[i]->priority_, cmd_vel_subs_[i]->timeout_);
    }
    else
    {
      NODELET_DEBUG_STREAM("CmdVelMux : no need to re-subscribe to input topic '" << cmd_vel_subs_[i]->topic_ << "'");
    }

    if (!cmd_vel_subs_[i]->timer_)
    {
      // Create (stopped by now) a one-shot timer for every subscriber, if it doesn't exist yet
      cmd_vel_subs_[i]->timer_ =
          pnh.createTimer(ros::Duration(cmd_vel_subs_[i]->timeout_), TimerFunctor(i, this), true, false);
    }

    if (cmd_vel_subs_[i]->timeout_ > longest_timeout)
    {
      longest_timeout = cmd_vel_subs_[i]->timeout_;
    }
  }

  if (!common_timer_)
  {
    // Create another timer for cmd_vel messages from any source, so we can
    // dislodge last active source if it gets stuck without further messages
    common_timer_period_ = longest_timeout * 2.0;
    common_timer_ =
        pnh.createTimer(ros::Duration(common_timer_period_), TimerFunctor(GLOBAL_TIMER, this), true, false);
  }
  else if (longest_timeout != (common_timer_period_ / 2.0))
  {
    // Longest timeout changed; just update existing timer period
    common_timer_period_ = longest_timeout * 2.0;
    common_timer_.setPeriod(ros::Duration(common_timer_period_));
  }

  NODELET_INFO_STREAM("CmdVelMux : (re)configured");
}

} // namespace cmd_vel_mux

PLUGINLIB_EXPORT_CLASS(cmd_vel_mux::CmdVelMux, nodelet::Nodelet);
