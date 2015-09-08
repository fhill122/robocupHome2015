Header header

#Request types
	uint8 CMD_DETECT = 0

uint8 cmd
string object
sensor_msgs/PointCloud2 cloud #point cloud of object for object classification

#EE camera being used
	string LEFT_CAM=left_hand_camera
	string RIGHT_CAM=right_hand_camera
string camera

#TO REMOVE ONCE VISION_MODULE IS UPDATED
bool enable
