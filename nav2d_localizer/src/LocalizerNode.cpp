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
	std::string laserTopic;
	n.param("laser_topic", laserTopic, std::string("scan"));
	ros::Subscriber laserSubscriber = n.subscribe(laserTopic, 100, &SelfLocalizer::process, &selfLocalizer);

	// Get the map via service call
	std::string mapService;
	n.param("map_service", mapService, std::string("get_map"));
	ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>(mapService);
	
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
