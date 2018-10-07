#include  "serializer.hpp"
#include  <iostream>

#define _USE_MATH_DEFINES
#include  <cmath>

RobotCommand::RobotCommand()
{
}

RobotCommand::RobotCommand(unsigned int id, float vel_norm, float vel_theta, float omega,
        float dribble_power,
        float kick_power, RobotCommand::KickType kick_type) :
    _id(id), _vel_norm(vel_norm), _vel_theta(vel_theta), _omega(omega),
    _dribble_power(dribble_power),
    _kick_power(kick_power), _kick_type(kick_type)
{}


/* Roots Protocol
 * 0: 1111 1111 |HEADER_1 0xFF
 * 1: 1100 0011 |HEADER_2 0xC3
 * 2: 0000 xxxx |x:ID
 * 3: aaaa aaaa |a:vel_norm
 * 4: bbbb bbbb |b:vel_theta
 * 5: cccc cccc |c:omega(0~254), 127->0, 254->2PI [rad/sec]
 * 6: d01e f110 |d:dribble_flag, e:kick_flag, f:chip_enable
 * 7: gggg hhhh |g:dribble_power, h:kick_power
 * 8: **** **** |XOR([2] ~ [7])
 * 9: **** **** |XOR([8],0xFF)
 *
 * ============================================================================
 *
 * x:ID
 *   description    : ロボットID
 *   protocol value : 0 ~ 15
 *   transform value: 0 ~ 15
 *   examples       : 0 -> No.0
 *
 * a:vel_norm
 *   description    : ロボット走行速度の大きさ
 *   protocol value : 0 ~ 254
 *   transform value: 0 ~ 4 m/sec
 *   examples       :
 *     0   -> 0 m/sec
 *     127 -> 2 m/sec
 *     254 -> 4 m/sec
 *
 * b:vel_theta
 *   description    : ロボット走行速度の方向
 *   protocol value : 0 ~ 180
 *   transform value: 0 ~ 2PI rad
 *   examples       :
 *     0   -> 0     rad
 *     45  -> PI/2  rad
 *     135 -> 3PI/2 rad
 *
 * c:omega
 *   description    : ロボット回転速度
 *   protocol value : 0 ~ 254
 *   transform value: -2PI ~ 2PI rad/sec
 *   examples       :
 *     0   -> -2PI rad/sec
 *     127 -> 0    rad/sec
 *     254 -> 2PI  rad/sec
 *
 * d:dribble_flag, e:kick_flag, f:chip_enable
 *   description    : ロボットコマンドフラグ
 *   protocol value : 0, 1
 *   transform value: true, false
 *   examples       :
 *     dribble_flag :0 -> dribble_flag :false
 *     kick_flag    :1 -> kick_flag    :true
 *
 * g:dribble_power, h:kick_power
 *   description    : ロボットコマンドパワー
 *   protocol value : 0 ~ 15
 *   transform value: 0 ~ MAX_POWER
 *   examples       :
 *     dribble_power:0  -> dribble_power:0
 *     kick_power   :15 -> kick_power   :MAX_POWER
 *
 */

const char RootsSerializer::HEADER_1 = 0xFF;
const char RootsSerializer::HEADER_2 = 0xC3;
const int RootsSerializer::NUM_DATA = 10;

bool  RootsSerializer::serialize(RobotCommand cmd, std::string* data)
{
    RobotCommand_Binary cmd_bin = scalingToBinary(cmd);
    data->resize(NUM_DATA);

    data->at(0) = HEADER_1;
    data->at(1) = HEADER_2;

    data->at(2) = cmd_bin._id;

    data->at(3) = cmd_bin._vel_norm;
    data->at(4) = cmd_bin._vel_theta;
    data->at(5) = cmd_bin._omega;

    data->at(6) = 0x00;
    // dribble_flag
    if (cmd_bin._dribble_power > 0) {
        data->at(6) |= 0x80;
    }
    data->at(6) |= 0x20; // magic number for HEADER_2
    // kick_flag
    if (cmd_bin._kick_power > 0) {
        data->at(6) |= 0x10;
    }
    // chip_enable
    if (cmd_bin._kick_type == RobotCommand_Binary::CHIP) {
        data->at(6) |= 0x08;
    }
    data->at(6) |= 0x04; // magic number for HEADER_2
    // TODO : ChargerFlag, ErrFlag
    data->at(6) |= 0x02;

    // TODO : Overflow err expression
    data->at(7) = 0x00;
    data->at(7) += cmd_bin._dribble_power;
    data->at(7) <<= 4;
    data->at(7) += cmd_bin._kick_power;

    // Make checksum
    data->at(8) = 0x00;
    for (size_t i=2; i<8; i++) {
        data->at(8) ^= data->at(i);
    }
    data->at(9) = data->at(8) ^ 0xFF;

    return  true;
}


// convert MKS unit to binary data in order to send as packet
RootsSerializer::RobotCommand_Binary RootsSerializer::scalingToBinary(RobotCommand robot_command)
{
    RobotCommand_Binary command_binary;

    // TODO:copy instances
    command_binary._id = robot_command._id;

    // Velocity Norm
    if (robot_command._vel_norm < 0.0) {
        command_binary._vel_norm = 0;
    } else {
        command_binary._vel_norm = robot_command._vel_norm * 255 / 4;
        if (command_binary._vel_norm > 255) {
            command_binary._vel_norm = 255;
        }
    }

    // Velcity angle
    robot_command._vel_theta = zeroTo2pi(robot_command._vel_theta);
    command_binary._vel_theta = int(round(robot_command._vel_theta * 180.0 / M_PI)) / 2;
    if (robot_command._vel_theta > 180) {
        command_binary._vel_theta = 180;
    } else if (robot_command._vel_theta < 0) {
        command_binary._vel_theta = 0;
    }

    // Angular velocity
    robot_command._omega = round(robot_command._omega * 128 / (2*M_PI)) + 127;
    if (robot_command._omega > 254) {
        command_binary._omega = 254;
    } else if (robot_command._omega < 0) {
        command_binary._omega = 0;
    } else {
        command_binary._omega = robot_command._omega;
    }

    // Dribble power
    if (robot_command._dribble_power > 15) {
        command_binary._dribble_power = 15;
    } else if (robot_command._dribble_power < 0) {
        command_binary._dribble_power = 0;
    } else {
        command_binary._dribble_power = robot_command._dribble_power;
    }

    // Kick power
    if (robot_command._kick_power > 15) {
        command_binary._kick_power = 15;
    }else if (robot_command._kick_power < 0) {
        command_binary._kick_power = 0;
    } else {
        command_binary._kick_power = robot_command._kick_power;
    }

    // Kick Type
    if (robot_command._kick_type == RobotCommand::CHIP) {
        command_binary._kick_type = RobotCommand_Binary::CHIP;
    } else {
        command_binary._kick_type = RobotCommand_Binary::STRAIGHT;
    }

    return  command_binary;
};


float RootsSerializer::piTopi(float angle)
{
    while (angle >=  M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return  angle;
}

float RootsSerializer::zeroTo2pi(float angle)
{
    while (angle >= 2*M_PI) angle -= 2*M_PI;
    while (angle < 0)      angle += 2*M_PI;
    return angle;
}
