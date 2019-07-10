#include <libsurvive/survive_api.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include "../vendor/libsurvive/redist/linmath.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "roslibsurvive_frame_publisher");
	SurviveSimpleContext *ctx = survive_simple_init(argc, argv);

	ros::NodeHandle nh;

	tf2_ros::TransformBroadcaster br;

	survive_simple_start_thread(ctx);

	while(survive_simple_is_running(ctx) && ros::ok())
	{
		SurvivePose pose;

		for (const auto *it = survive_simple_get_next_updated(ctx); it != nullptr;
		     it = survive_simple_get_next_updated(ctx)) {
			const char *name = survive_simple_object_name(it);
			geometry_msgs::TransformStamped transform;

			uint32_t timestamp = survive_simple_object_get_latest_pose(it, &pose);

			transform.header.frame_id = "map"; // TODO: set name as a parameter
			transform.child_frame_id = name;

			transform.transform.translation.x = pose.Pos[0];
			transform.transform.translation.y = pose.Pos[1];
			transform.transform.translation.z = pose.Pos[2];

			transform.transform.rotation.w = pose.Rot[0];
			transform.transform.rotation.x = pose.Rot[1];
			transform.transform.rotation.y = pose.Rot[2];
			transform.transform.rotation.z = pose.Rot[3];

			br.sendTransform(transform);
		}

		ros::spinOnce();
	}

	survive_simple_close(ctx);
	return 0;
}
