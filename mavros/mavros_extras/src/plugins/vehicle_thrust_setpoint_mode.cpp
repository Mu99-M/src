/**
 * @brief Vehicle Thrust Setpoint Plugin
 * @file vehicle_thrust_setpoint_mode.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>
#include <mavros_msgs/VehicleThrustSetpointMode.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vehicle Thrust Setpoint Plugin
 */
class VehicleThrustSetpointModePlugin : public plugin::PluginBase {
public:
	VehicleThrustSetpointModePlugin() : PluginBase(),
		nh("~vehicle_thrust_setpoint_mode")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		vehicle_thrust_setpoint_mode_sub = nh.subscribe("vehicle_thrust_setpoint_sub", 1000, &VehicleThrustSetpointModePlugin::vehicle_thrust_setpoint_mode_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {};
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber vehicle_thrust_setpoint_mode_sub;

	void vehicle_thrust_setpoint_mode_cb(const mavros_msgs::VehicleThrustSetpointMode &msg)
	{
		mavlink::common::msg::VEHICLE_THRUST_SETPOINT_MODE vts{};

		vts.xyz[0] = msg.xyz[0];
		vts.xyz[1] = msg.xyz[1];
		vts.xyz[2] = msg.xyz[2];

		UAS_FCU(m_uas)->send_message_ignore_drop(vts);
		}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VehicleThrustSetpointModePlugin, mavros::plugin::PluginBase)
