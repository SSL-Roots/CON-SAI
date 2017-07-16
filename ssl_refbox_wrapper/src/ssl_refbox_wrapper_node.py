#!/usr/bin/env  python
import  rospy
import  referee_pb2
import  multicast

from    std_msgs.msg import Int8
from    std_msgs.msg import Duration


if  __name__ == '__main__':
    rospy.init_node('ssl_refbox_wrapper_node')

    our_color = rospy.get_param('/friend_color', 'yellow').upper()
    multicast_addr = rospy.get_param('~multicast_addr', '224.5.23.13')
    multicast_port = rospy.get_param('~multicast_port', 10003)

    #make protobuf instance
    protobuf = referee_pb2.SSL_Referee()
    sock    = multicast.Multicast(multicast_addr, multicast_port)

    pub_stage       = rospy.Publisher('~stage', Int8, queue_size = 10)
    pub_stage_duration  = rospy.Publisher('~stage_duration', Duration, queue_size = 10)
    pub_command = rospy.Publisher('~command', Int8, queue_size = 10)
    pub_command_duration   =  rospy.Publisher('~command_duration', Duration, queue_size = 10)

    while not   rospy.is_shutdown():
        buf    = sock.recv(1024)

        protobuf.ParseFromString(buf)

        pub_stage.publish(Int8(protobuf.stage));
        # pub_stage_duration.publish(Duration());
        pub_command.publish(Int8(protobuf.command));
        # pub_command_duration.publish(Int8(protobuf.stage);
