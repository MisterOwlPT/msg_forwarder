#!/usr/bin/python3

import rospy
import traceback
from msg_forwarder.msg_forwarder_class import MsgForwarder


if __name__ == "__main__":

    rospy.init_node('msg_forwarder')

    try:
        rosbridge_host = rospy.get_param('~rosbridge_host')
        rosbridge_port = rospy.get_param('~rosbridge_port')
        topics_file = rospy.get_param('~topics_file')
    except Exception as e:
        raise KeyError(f'Unable to access ROS parameter server: {e}')

    try:
        if not topics_file:
            raise Exception('No list of topics was provided. Specify YAML file with <topics_file>.')

        MsgForwarder(topics_file, rosbridge_host, rosbridge_port)
        rospy.spin()

    except Exception as e:

        rospy.logerr('[MsgForwarder] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())
        quit()
