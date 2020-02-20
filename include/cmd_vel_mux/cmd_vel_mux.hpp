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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{

/*****************************************************************************
 ** CmdVelMux
 *****************************************************************************/

struct ParameterValues
{
  std::string topic;       /**< The name of the topic */
  double timeout{-1.0};    /**< Timer's timeout, in seconds  */
  int64_t priority{-1};    /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
  std::string short_desc;  /**< Short description */
};

bool operator == (const ParameterValues & parameters1, const ParameterValues & parameters2)
{
  if (parameters1.topic != parameters2.topic)
  {
    return false;
  }
  else if (parameters1.timeout != parameters2.timeout)
  {
    return false;
  }
  else if (parameters1.priority != parameters2.priority)
  {
    return false;
  }
  else if (parameters1.short_desc != parameters2.short_desc)
  {
    return false;
  }
  return true;
}

class CmdVelMux final : public rclcpp::Node
{
public:
  explicit CmdVelMux(rclcpp::NodeOptions options);
  ~CmdVelMux() override = default;
  CmdVelMux(CmdVelMux && c) = delete;
  CmdVelMux & operator=(CmdVelMux && c) = delete;
  CmdVelMux(const CmdVelMux & c) = delete;
  CmdVelMux & operator=(const CmdVelMux & c) = delete;

private:
  static const std::string VACANT;  /**< ID for "nobody" active input;*/

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_topic_pub_;   /**< Multiplexed command velocity topic */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_subscriber_pub_;  /**< Currently allowed cmd_vel subscriber */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::string allowed_;

  void timerCallback(const std::string & key);
  void cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg, const std::string & key);

  rcl_interfaces::msg::SetParametersResult parameterUpdate(
    const std::vector<rclcpp::Parameter> & update_parameters);

  /*********************
   ** Private Classes
   **********************/

  /**
   * Inner class describing an individual subscriber to a cmd_vel topic
   */
  struct CmdVelSub final
  {
    std::string            name_;         /**< Descriptive name; must be unique to this subscriber */
    ParameterValues        values_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        sub_;         /**< The subscriber itself */
    rclcpp::TimerBase::SharedPtr             timer_;        /**< No incoming messages timeout */
  };

  bool addInputToParameterMap(std::map<std::string, ParameterValues> & parsed_parameters, const std::string & input_name, const std::string & parameter_name, const rclcpp::Parameter & parameter_value);
  bool parametersAreValid(const std::map<std::string, ParameterValues> & parameters) const;
  void configureFromParameters(const std::map<std::string, ParameterValues> & parameters);
  std::map<std::string, ParameterValues> parseFromParametersMap(const std::map<std::string, rclcpp::Parameter> & parameters);

  std::map<std::string, std::shared_ptr<CmdVelSub>> map_;
};

} // namespace cmd_vel_mux

#endif /* CMD_VEL_MUX__CMD_VEL_MUX_HPP_ */
