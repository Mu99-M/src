/**
 * @brief Vehicle Status Plugin
 * @file ./vehicle_status/vehicle_status.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>

#include <mavros_msgs/VehicleStatus.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vehicle Status Plugin
 */
class VehicleStatusPlugin : public plugin::PluginBase {
public:
	VehicleStatusPlugin() : PluginBase(),
		nh("~vehicle_status")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		vehicle_status_pub = nh.advertise<mavros_msgs::VehicleStatus>("vehicle_status", 1000);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&VehicleStatusPlugin::handle_vehicle_status),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher vehicle_status_pub;

	void handle_vehicle_status(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VEHICLE_STATUS &vehicle_status)
	{
		auto vmsg = boost::make_shared<mavros_msgs::VehicleStatus>();

		vmsg->header.stamp = ros::Time::now();

		vmsg->armed_time = vehicle_status.armed_time;
		vmsg->takeoff_time = vehicle_status.takeoff_time;
		vmsg->arming_state = vehicle_status.arming_state;
		vmsg->latest_arming_reason = vehicle_status.latest_arming_reason;
		vmsg->latest_disarming_reason = vehicle_status.latest_disarming_reason;
		vmsg->nav_state_timestamp = vehicle_status.nav_state_timestamp;
		vmsg->nav_state_user_intention = vehicle_status.nav_state_user_intention;
		vmsg->nav_state = vehicle_status.nav_state;
		vmsg->failure_detector_status = vehicle_status.failure_detector_status;
		vmsg->hil_state = vehicle_status.hil_state;
		vmsg->vehicle_type = vehicle_status.vehicle_type;
		vmsg->failsafe = vehicle_status.failsafe;
		vmsg->failsafe_and_user_took_over = vehicle_status.failsafe_and_user_took_over;
		vmsg->gcs_connection_lost = vehicle_status.gcs_connection_lost;
		vmsg->gcs_connection_lost_counter = vehicle_status.gcs_connection_lost_counter;
		vmsg->high_latency_data_link_lost = vehicle_status.high_latency_data_link_lost;
		vmsg->is_vtol = vehicle_status.is_vtol;
		vmsg->is_vtol_tailsitter = vehicle_status.is_vtol_tailsitter;
		vmsg->in_transition_mode = vehicle_status.in_transition_mode;
		vmsg->in_transition_to_fw = vehicle_status.in_transition_to_fw;

		vehicle_status_pub.publish(vmsg);
		}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VehicleStatusPlugin, mavros::plugin::PluginBase)
