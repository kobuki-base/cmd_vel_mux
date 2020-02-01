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
#include "cmd_vel_mux/cmd_vel_subscribers.hpp"

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
  static const unsigned int GLOBAL_TIMER = 888888;  /**< ID for the global timer functor; anything big is ok */

  CmdVelSubscribers cmd_vel_subs_;    /**< Pool of cmd_vel topics subscribers */
  ros::Publisher output_topic_pub_;   /**< Multiplexed command velocity topic */
  ros::Publisher active_subscriber_;  /**< Currently allowed cmd_vel subscriber */
  ros::Timer common_timer_;           /**< No messages from any subscriber timeout */
  double common_timer_period_;        /**< No messages from any subscriber timeout period */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);

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
};

} // namespace cmd_vel_mux

#endif /* CMD_VEL_MUX__CMD_VEL_MUX_HPP_ */
