/**
 * @brief Actuator Outputs Plugin
 * @file ./actuator_outputs/actuator_outputs.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ActuatorOutputs.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Actuator Outputs Plugin
 */
class ActuatorOutputsPlugin : public plugin::PluginBase {
public:
	ActuatorOutputsPlugin() : PluginBase(),
		nh("~actuator_outputs")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		actuator_outputs_pub = nh.advertise<mavros_msgs::ActuatorOutputs>("actuator_outputs", 1000);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&ActuatorOutputsPlugin::handle_actuator_outputs),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher actuator_outputs_pub;

	void handle_actuator_outputs(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ACTUATOR_OUTPUTS &actuator_outputs)
	{
		auto vmsg = boost::make_shared<mavros_msgs::ActuatorOutputs>();

		vmsg->header.stamp = ros::Time::now();

		vmsg->noutputs = actuator_outputs.noutputs;
		vmsg->output[0] = actuator_outputs.output[0];
		vmsg->output[1] = actuator_outputs.output[1];
		vmsg->output[2] = actuator_outputs.output[2];
		vmsg->output[3] = actuator_outputs.output[3];
		vmsg->output[4] = actuator_outputs.output[4];
		vmsg->output[5] = actuator_outputs.output[5];
		vmsg->output[6] = actuator_outputs.output[6];
		vmsg->output[7] = actuator_outputs.output[7];
		vmsg->output[8] = actuator_outputs.output[8];
		vmsg->output[9] = actuator_outputs.output[9];
		vmsg->output[10] = actuator_outputs.output[10];
		vmsg->output[11] = actuator_outputs.output[11];
		vmsg->output[12] = actuator_outputs.output[12];
		vmsg->output[13] = actuator_outputs.output[13];
		vmsg->output[14] = actuator_outputs.output[14];
		vmsg->output[15] = actuator_outputs.output[15];

		actuator_outputs_pub.publish(vmsg);
		}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ActuatorOutputsPlugin, mavros::plugin::PluginBase)
