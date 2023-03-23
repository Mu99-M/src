/**
 * @brief Vehicle Control Mode Plugin
 * @file ./vehicle_control_mode/vehicle_control_mode.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>

#include <mavros_msgs/VehicleControlMode.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vehicle Control Mode Plugin
 */
class VehicleControlModePlugin : public plugin::PluginBase {
public:
	VehicleControlModePlugin() : PluginBase(),
		nh("~vehicle_control_mode")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		vehicle_control_mode_pub = nh.advertise<mavros_msgs::VehicleControlMode>("vehicle_control_mode", 1000);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&VehicleControlModePlugin::handle_vehicle_control_mode),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher vehicle_control_mode_pub;

	void handle_vehicle_control_mode(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VEHICLE_CONTROL_MODE &vehicle_control_mode)
	{
		auto vmsg = boost::make_shared<mavros_msgs::VehicleControlMode>();

		vmsg->header.stamp = ros::Time::now();

		vmsg->flag_armed = vehicle_control_mode.flag_armed;
		vmsg->flag_multicopter_position_control_enabled = vehicle_control_mode.flag_multicopter_position_control_enabled;
		vmsg->flag_control_manual_enabled = vehicle_control_mode.flag_control_manual_enabled;
		vmsg->flag_control_auto_enabled = vehicle_control_mode.flag_control_auto_enabled;
		vmsg->flag_control_offboard_enabled = vehicle_control_mode.flag_control_offboard_enabled;
		vmsg->flag_control_rates_enabled = vehicle_control_mode.flag_control_rates_enabled;
		vmsg->flag_control_attitude_enabled = vehicle_control_mode.flag_control_attitude_enabled;
		vmsg->flag_control_acceleration_enabled = vehicle_control_mode.flag_control_acceleration_enabled;
		vmsg->flag_control_velocity_enabled = vehicle_control_mode.flag_control_velocity_enabled;
		vmsg->flag_control_position_enabled = vehicle_control_mode.flag_control_position_enabled;
		vmsg->flag_control_altitude_enabled = vehicle_control_mode.flag_control_altitude_enabled;
		vmsg->flag_control_climb_rate_enabled = vehicle_control_mode.flag_control_climb_rate_enabled;
		vmsg->flag_control_termination_enabled = vehicle_control_mode.flag_control_termination_enabled;

		vehicle_control_mode_pub.publish(vmsg);
		}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VehicleControlModePlugin, mavros::plugin::PluginBase)
