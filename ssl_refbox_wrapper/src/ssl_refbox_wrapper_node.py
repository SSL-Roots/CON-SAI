#!/usr/bin/env  python
import rospy
import referee_pb2
import multicast

from std_msgs.msg import Int8
from std_msgs.msg import Duration
from consai_msgs.msg import RefereeTeamInfo


def convert_team_info(data):
    team_info = RefereeTeamInfo()

    team_info.name = str(data.name)
    team_info.score = data.score
    team_info.red_cards = data.red_cards
    team_info.yellow_card_times = data.yellow_card_times
    team_info.yellow_cards = data.yellow_cards
    team_info.timeouts = data.timeouts
    team_info.timeout_time = data.timeout_time
    team_info.goalie = data.goalie

    return team_info


if  __name__ == '__main__':
    rospy.init_node('ssl_refbox_wrapper_node')

    our_color = rospy.get_param('friend_color', 'yellow').upper()
    multicast_addr = rospy.get_param('~multicast_addr', '224.5.23.13')
    multicast_port = rospy.get_param('~multicast_port', 10003)

    #make protobuf instance
    protobuf = referee_pb2.SSL_Referee()
    sock    = multicast.Multicast(multicast_addr, multicast_port)

    pub_stage       = rospy.Publisher('~stage', Int8, queue_size = 10)
    pub_stage_duration  = rospy.Publisher('~stage_duration', Duration, queue_size = 10)
    pub_command = rospy.Publisher('~command', Int8, queue_size = 10)
    pub_command_duration   =  rospy.Publisher('~command_duration', Duration, queue_size = 10)
    pub_blue_info = rospy.Publisher('~blue_info', RefereeTeamInfo, queue_size = 10)
    pub_yellow_info = rospy.Publisher('~yellow_info', RefereeTeamInfo, queue_size = 10)

    r = rospy.Rate(60)
    while not   rospy.is_shutdown():
        buf    = sock.recv(1024)

        if buf == None:
            r.sleep()
            continue

        protobuf.ParseFromString(buf)

        pub_stage.publish(Int8(protobuf.stage));
        # pub_stage_duration.publish(Duration());
        pub_command.publish(Int8(protobuf.command));
        # pub_command_duration.publish(Int8(protobuf.stage);

        blue_info = convert_team_info(protobuf.blue)
        pub_blue_info.publish(blue_info)

        yellow_info = convert_team_info(protobuf.yellow)
        pub_yellow_info.publish(yellow_info)

