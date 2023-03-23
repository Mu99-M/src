/**
 * @brief Vehicle Torque Setpoint Plugin
 * @file vehicle_torque_setpoint_mode.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>
#include <mavros_msgs/VehicleTorqueSetpointMode.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vehicle Torque Setpoint Plugin
 */
class VehicleTorqueSetpointModePlugin : public plugin::PluginBase {
public:
	VehicleTorqueSetpointModePlugin() : PluginBase(),
		nh("~vehicle_torque_setpoint_mode")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		vehicle_torque_setpoint_mode_sub = nh.subscribe("vehicle_torque_setpoint_sub", 10, &VehicleTorqueSetpointModePlugin::vehicle_torque_setpoint_mode_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {};
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber vehicle_torque_setpoint_mode_sub;

	void vehicle_torque_setpoint_mode_cb(const mavros_msgs::VehicleTorqueSetpointMode &msg)
	{
		mavlink::common::msg::VEHICLE_TORQUE_SETPOINT_MODE vts{};

		vts.xyz[0] = msg.xyz[0];
		vts.xyz[1] = msg.xyz[1];
		vts.xyz[2] = msg.xyz[2];

		UAS_FCU(m_uas)->send_message_ignore_drop(vts);
		}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VehicleTorqueSetpointModePlugin, mavros::plugin::PluginBase)
