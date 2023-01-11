#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/GetMap.h"

#include "nav2d_localizer/SelfLocalizer.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "localize_node");
	ros::NodeHandle n;

	SelfLocalizer selfLocalizer(true);

	tf::TransformBroadcaster tfBC;
	
	// Subscribe to laser scans
	ros::Subscriber laserSubscriber = n.subscribe("scan", 100, &SelfLocalizer::process, &selfLocalizer);

	// Get the map via service call
	ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("get_map");
	
	nav_msgs::GetMap srv;
	mapClient.waitForExistence();
	if(mapClient.call(srv))
	{
		selfLocalizer.convertMap(srv.response.map);
		selfLocalizer.initialize();
	}else
	{
		ROS_FATAL("Could not get a map.");
		return 1;
	}

	ros::Rate loopRate(10);
	while(ros::ok())
	{
		tfBC.sendTransform(selfLocalizer.getMapToOdometry());
		
		ros::spinOnce();
		loopRate.sleep();
	}

	ros::spin();
	return 0;
}
