/**
 * @file /include/yocs_cmd_vel_mux/cmd_vel_mux_nodelet.hpp
 *
 * @brief Structure for the yocs_cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef CMD_VEL_MUX__CMD_VEL_MUX_HPP_
#define CMD_VEL_MUX__CMD_VEL_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>

#include "cmd_vel_mux/reloadConfig.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{

/*****************************************************************************
 ** CmdVelMux
 *****************************************************************************/

class CmdVelMux final : public nodelet::Nodelet
{
public:
  virtual void onInit();

  CmdVelMux();

  ~CmdVelMux();

private:
  static const unsigned int VACANT       = 666666;  /**< ID for "nobody" active input; anything big is ok */

  ros::Publisher output_topic_pub_;   /**< Multiplexed command velocity topic */
  ros::Publisher active_subscriber_;  /**< Currently allowed cmd_vel subscriber */
  ros::Timer common_timer_;           /**< No messages from any subscriber timeout */
  double common_timer_period_;        /**< No messages from any subscriber timeout period */

  void commonTimerCallback(const ros::TimerEvent& event);
  void timerCallback(unsigned int idx);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);

  unsigned int allowed_;

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<cmd_vel_mux::reloadConfig> * dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<cmd_vel_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb_;
  void reloadConfiguration(cmd_vel_mux::reloadConfig &config, uint32_t level);

  /*********************
   ** Private Classes
   **********************/

  // Functor assigned to each incoming velocity topic to bind it to cmd_vel callback
  class CmdVelFunctor;

  // Functor assigned to each velocity messages source to bind it to timer callback
  class TimerFunctor;

  /**
   * Inner class describing an individual subscriber to a cmd_vel topic
   */
  struct CmdVelSub final
  {
    std::string            name_;         /**< Descriptive name; must be unique to this subscriber */
    std::string            topic_;        /**< The name of the topic */
    ros::Subscriber        sub_;         /**< The subscriber itself */
    ros::Timer             timer_;        /**< No incoming messages timeout */
    double                 timeout_;      /**< Timer's timeout, in seconds  */
    unsigned int           priority_;     /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string            short_desc_;   /**< Short description (optional) */
  };

  /**
   * @brief Configures the subscribers from a yaml file.
   *
   * @exception YamlException : problem parsing the yaml
   * @exception EmptyCfgException : empty configuration file
   * @param node : node holding all the subscriber configuration
   */
  void configure(const YAML::Node& node);

  std::vector<std::shared_ptr<CmdVelSub>> list_;
};

} // namespace cmd_vel_mux

#endif /* CMD_VEL_MUX__CMD_VEL_MUX_HPP_ */
