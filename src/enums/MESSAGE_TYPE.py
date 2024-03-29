__package__ = 'enums'
class MESSAGE_TYPE(object):
    ROSLINK_MESSAGE_HEARTBEAT           = 0
    ROSLINK_MESSAGE_ROBOT_STATUS        = 1
    ROSLINK_MESSAGE_GLOBAL_MOTION       = 2
    ROSLINK_MESSAGE_GPS_RAW_INFO        = 3
    ROSLINK_MESSAGE_RANGE_FINDER_DATA   = 4
    ROSLINK_MESSAGE_ROSLINK_IMAGE       = 5
    ROSLINK_MESSAGE_ROSLINK_MAP	        = 6
    ROSLINK_MESSAGE_ROSLINK_GEO_TAGED_IMAGE       = 7

    ROSLINK_MESSAGE_COMMAND_TWIST           =100
    ROSLINK_MESSAGE_COMMAND_GO_TO_WAYPOINT  = 101
    ROSLINK_MESSAGE_COMMAND_TAKEOFF         = 102
    ROSLINK_MESSAGE_COMMAND_LAND   		    = 103
    ROSLINK_MESSAGE_COMMAND_ARM            = 104
    ROSLINK_MESSAGE_COMMAND_DISARM            = 105
    ROSLINK_MESSAGE_COMMAND_SET_MODE            = 106
    ROSLINK_MESSAGE_COMMAND_GET_MAP	    	= 107
    ROSLINK_MESSAGE_COMMAND_ESTOP               = 108



    # MAVlink messages 
    MAVLINK_MESSAGE_HEARTBEAT           = 0;
    MAVLINK_GPS_RAW_INT                 = 24;
