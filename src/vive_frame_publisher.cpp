#include <libsurvive/survive_api.h>
#include <unordered_map>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


std::string base_frame_id;
double yaw, pitch, roll, yaw2, pitch2, roll2, yaw_static, pitch_static, roll_static;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "roslibsurvive_frame_publisher");
	SurviveSimpleContext *ctx = survive_simple_init(argc, argv);

	ros::NodeHandle nh("~");
	nh.param<std::string>("base_frame_id", base_frame_id, "map");
	nh.param<double>("yaw", yaw, 0.0);
	nh.param<double>("pitch", pitch, 0.0);
	nh.param<double>("roll", roll, 0.0);
	nh.param<double>("yaw_static", yaw_static, 0.0);
	nh.param<double>("pitch_static", pitch_static, 0.0);
	nh.param<double>("roll_static", roll_static, 0.0);
	//ROS_INFO("Got angles: %g, %g, %g", yaw, pitch, roll);
	std::unordered_map<std::string, ros::Publisher> pubTopics;

	tf2_ros::TransformBroadcaster br;
	geometry_msgs::PoseStamped poseMsg;
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
	tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_Listener(tf_buffer);

	//std::string base_frame_id = "base_link"; // TODO: pass base frame ID as a parameter

	survive_simple_start_thread(ctx);

    tf2::Quaternion q_orig, q_rot, q_rot2, q_new, q_static;
	ros::Publisher T20_t1 = nh.advertise<geometry_msgs::PoseStamped>("T20_pose_t1", 1);
	ros::Publisher T20_t2 = nh.advertise<geometry_msgs::PoseStamped>("T20_pose_t2", 1);
	geometry_msgs::TransformStamped transform, vive_transform;
	ros::Rate pub_rate(30);

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

	//uint32_t seq = 1;

	while(survive_simple_is_running(ctx) && ros::ok())
	{
		SurvivePose pose;

		for (const auto *it = survive_simple_get_next_updated(ctx); it != nullptr;
		     it = survive_simple_get_next_updated(ctx)) {
			const char *name = survive_simple_object_name(it);
			std::string str_name = name;
			std::string t20_name = "T20";
			uint32_t timestamp = survive_simple_object_get_latest_pose(it, &pose);

			q_orig = tf2::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]);
			q_orig.normalize(); 
			
			// Stuff the new rotation back into the pose. This requires conversion into a msg type
			poseMsg.header.frame_id = base_frame_id;
			poseMsg.pose.position.x = pose.Pos[0];
			poseMsg.pose.position.y = pose.Pos[1];
			poseMsg.pose.position.z = pose.Pos[2];
			tf2::convert(q_orig, poseMsg.pose.orientation);

			transform.header.frame_id = base_frame_id;
			transform.child_frame_id = name;
			transform.transform.translation.x = pose.Pos[0];
			transform.transform.translation.y = pose.Pos[1];
			transform.transform.translation.z = pose.Pos[2];
			tf2::convert(q_orig, transform.transform.rotation);

			transform.header.stamp = ros::Time::now();
			poseMsg.header.stamp = transform.header.stamp;
			br.sendTransform(transform);
			getPublisher(name).publish(poseMsg);

			if (str_name.compare(t20_name)==0) {
				static_transformStamped.header.stamp = transform.header.stamp;
  				static_transformStamped.header.frame_id = str_name;
  				static_transformStamped.child_frame_id = "T20_t1";
  				static_transformStamped.transform.translation.x = 0;
  				static_transformStamped.transform.translation.y = 0;
 				static_transformStamped.transform.translation.z = 0;
  				q_static.setRPY(roll_static, pitch_static, yaw_static);
				tf2::convert(q_static, static_transformStamped.transform.rotation);
				static_broadcaster.sendTransform(static_transformStamped);
				
				try {
					vive_transform = tf_buffer.lookupTransform(base_frame_id, "T20_t1", static_transformStamped.header.stamp, ros::Duration(0.02));
					poseMsg.header.frame_id = base_frame_id;
					poseMsg.pose.position.x = vive_transform.transform.translation.x;
					poseMsg.pose.position.y = vive_transform.transform.translation.y;
					poseMsg.pose.position.z = vive_transform.transform.translation.z;
					poseMsg.pose.orientation = vive_transform.transform.rotation;
					T20_t1.publish(poseMsg);
				} catch (tf2::TransformException &ex) {
    				ROS_WARN("Could NOT transform: %s", ex.what());
  				}
			}
		}
		ros::spinOnce();
		pub_rate.sleep();

	}

	survive_simple_close(ctx);
	return 0;
}
