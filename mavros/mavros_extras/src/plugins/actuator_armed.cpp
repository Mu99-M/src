/**
 * @brief Actuator Armed Plugin
 * @file ./actuator_armed/actuator_armed.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ActuatorArmed.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Actuator Armed Plugin
 */
class ActuatorArmedPlugin : public plugin::PluginBase {
public:
	ActuatorArmedPlugin() : PluginBase(),
		nh("~actuator_armed")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		actuator_armed_pub = nh.advertise<mavros_msgs::ActuatorArmed>("actuator_armed", 1000);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&ActuatorArmedPlugin::handle_actuator_armed),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher actuator_armed_pub;

	void handle_actuator_armed(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ACTUATOR_ARMED &actuator_armed)
	{
		auto vmsg = boost::make_shared<mavros_msgs::ActuatorArmed>();

		vmsg->header.stamp = ros::Time::now();

		vmsg->armed = actuator_armed.armed;
		vmsg->prearmed = actuator_armed.prearmed;
		vmsg->ready_to_arm = actuator_armed.ready_to_arm;
		vmsg->lockdown = actuator_armed.lockdown;
		vmsg->manual_lockdown = actuator_armed.manual_lockdown;
		vmsg->force_failsafe = actuator_armed.force_failsafe;
		vmsg->in_esc_calibration_mode = actuator_armed.in_esc_calibration_mode;

		actuator_armed_pub.publish(vmsg);
		}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ActuatorArmedPlugin, mavros::plugin::PluginBase)
