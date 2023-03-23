/**
 * @brief Vehicle Attitude plugin
 * @file vehicle_attitude.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>

#include <mavros_msgs/VehicleAttitude.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vehicle Attitude plugin
 */
class VehicleAttitudePlugin : public plugin::PluginBase {
public:
	VehicleAttitudePlugin() : PluginBase(),
		nh("~vehicle_attitude")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		vehicle_attitude_pub = nh.advertise<mavros_msgs::VehicleAttitude>("vehicle_attitude", 1000);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&VehicleAttitudePlugin::handle_vehicle_attitude),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher vehicle_attitude_pub;

	void handle_vehicle_attitude(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VEHICLE_ATTITUDE &vehicle_attitude)
	{
		auto vmsg = boost::make_shared<mavros_msgs::VehicleAttitude>();
		vmsg->header.stamp = ros::Time::now();

		vmsg->q_w = vehicle_attitude.q_w;
		vmsg->q_x = vehicle_attitude.q_x;
		vmsg->q_y = vehicle_attitude.q_y;
		vmsg->q_z = vehicle_attitude.q_z;

		vmsg->delta_q_w = vehicle_attitude.delta_q_w;
		vmsg->delta_q_x = vehicle_attitude.delta_q_x;
		vmsg->delta_q_y = vehicle_attitude.delta_q_y;
		vmsg->delta_q_z = vehicle_attitude.delta_q_z;

		vehicle_attitude_pub.publish(vmsg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VehicleAttitudePlugin, mavros::plugin::PluginBase)
