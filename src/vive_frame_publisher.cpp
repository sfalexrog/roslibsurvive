#include <libsurvive/survive_api.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <unordered_map>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "roslibsurvive_frame_publisher");
	SurviveSimpleContext *ctx = survive_simple_init(argc, argv);

	ros::NodeHandle nh;
	std::unordered_map<std::string, ros::Publisher> pubTopics;

	tf2_ros::TransformBroadcaster br;

	std::string baseFrameId = "map"; // TODO: pass base frame ID as a parameter

	survive_simple_start_thread(ctx);

	auto getPublisher = [&](const char *name) -> ros::Publisher& {
		std::string sName = name;
		auto it = pubTopics.find(sName);
		if (it != pubTopics.end()) {
			return it->second;
		}

		ROS_INFO("Adding publisher for for %s", name);
		// Latch poses for lighthouses
		bool isLighthouse = (sName.find("LH") != std::string::npos);
		pubTopics[sName] = nh.advertise<geometry_msgs::PoseStamped>(sName + "_pose", 1, isLighthouse);
		return pubTopics[sName];
	};

	while(survive_simple_is_running(ctx) && ros::ok())
	{
		SurvivePose pose;

		for (const auto *it = survive_simple_get_next_updated(ctx); it != nullptr;
		     it = survive_simple_get_next_updated(ctx)) {
			const char *name = survive_simple_object_name(it);
			geometry_msgs::TransformStamped transform;

			uint32_t timestamp = survive_simple_object_get_latest_pose(it, &pose);

			transform.header.frame_id = baseFrameId;
			transform.header.stamp = ros::Time::now();
			transform.child_frame_id = name;

			transform.transform.translation.x = pose.Pos[0];
			transform.transform.translation.y = pose.Pos[1];
			transform.transform.translation.z = pose.Pos[2];

			transform.transform.rotation.w = pose.Rot[0];
			transform.transform.rotation.x = pose.Rot[1];
			transform.transform.rotation.y = pose.Rot[2];
			transform.transform.rotation.z = pose.Rot[3];

			br.sendTransform(transform);

			geometry_msgs::PoseStamped poseMsg;
			poseMsg.header.frame_id = baseFrameId;
			poseMsg.pose.position.x = pose.Pos[0];
			poseMsg.pose.position.y = pose.Pos[1];
			poseMsg.pose.position.z = pose.Pos[2];

			poseMsg.pose.orientation.w = pose.Rot[0];
			poseMsg.pose.orientation.x = pose.Rot[1];
			poseMsg.pose.orientation.y = pose.Rot[2];
			poseMsg.pose.orientation.z = pose.Rot[3];

			getPublisher(name).publish(poseMsg);
		}

		ros::spinOnce();
	}

	survive_simple_close(ctx);
	return 0;
}
