#include  <iostream>
#include  <string>
#include  <vector>
#include  <cstdlib>
#include  <stdio.h>

#define _USE_MATH_DEFINES
#include  <cmath>

#include  "../serializer.hpp"

#include  "gtest/gtest.h"

// MCT Susanoo Logic Radio Protocol Ver Roots
// 2016/01/09
//
// =====================================
//
// Byte |Abstract           | Type   | Value |
// -------------------------------------
// 00   | FrameHeader 1     | uint8  | 0x7F
// 01   | FrameHeader 2     | uint8  | 0x80
// 02   |　Machine ID        | uint8  | 1~255 |
// 03   | Velocity Norm     | uint8  | 0~255 | 0 - 4[m/s]
// 04   | Velocity Angle    | uint8  | 0~180 | 2[deg/value]
// 05   | Anguler Velocity  | uint8  | 0~255 | 127 -> 0[rad/s], 0 -> -2pi[rad/s], 254 -> 2pi[rad/s]
// 06   | Action of Machine | bitset | 7:dribble, 5,6:Unuse, 4:Kick Flag, 3:Kick Type(0:Straight, 1:Chip), 2:Unuse, 1:ChargerFlag, 0:ErrCheck(unuse),
// 07   | Power             | uint8  | 7-4:Dribble, 3-0:Kick
// 08   | Checksum 1        | uint8  | XOR 02-07byte
// 09   | Checksum 2        | uint8  | NOT of 08byte

void test_velocity(float v_norm, float v_theta, float omega, std::string* data)
{
  RootsSerializer rs;
  RobotCommand  cmd(0, v_norm, v_theta, omega, 0, 0, RobotCommand::STRAIGHT);
  rs.serialize(cmd, data);
}

TEST(Header, header_value)
{
  RootsSerializer rs;
  RobotCommand  cmd(0, 0, 0, 0, 0, 0, RobotCommand::STRAIGHT);

  std::string data;
  rs.serialize(cmd, &data);

  // HEADER
  EXPECT_EQ(char(0x7F), data.at(0));
  EXPECT_EQ(char(0x80), data.at(1));
}


//====== Velocity test =====
TEST(Velocity_value, all_zero)
{
  std::string data;
  test_velocity(0, 0, 0, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

TEST(Velocity_value, 2mps)
{
  std::string data;
  test_velocity(2, 0, 0, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(127), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

TEST(Velocity_value, norm_overflow)
{
  std::string data;
  test_velocity(100, 0, 0, &data);

  EXPECT_EQ(char(255), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

TEST(Velocity_value, norm_underflow)
{
  std::string data;
  test_velocity(-100, 0, 0, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

TEST(Velocity_value, theta_plus)
{
  std::string data;
  test_velocity(0, M_PI/2, 0, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(45), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

TEST(Velocity_value, theta_minus)
{
  std::string data;
  test_velocity(0, -M_PI/2, 0, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(135), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

TEST(Velocity_value, theta_2pi)
{
  std::string data;
  test_velocity(0, 2*M_PI, 0, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

TEST(Velocity_value, theta_overflow)
{
  std::string data;
  test_velocity(0, 3*M_PI, 0, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(90), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

TEST(Velocity_value, theta_underflow)
{
  std::string data;
  test_velocity(0, -3*M_PI, 0, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(90), data.at(4));
  EXPECT_EQ(char(127), data.at(5));
}

// =======================
// ===== Omega Test ======
// =======================
TEST(Velocity_value, omega_plus)
{
  std::string data;
  test_velocity(0, 0, M_PI, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(191), data.at(5));
}

TEST(Velocity_value, omega_minus)
{
  std::string data;
  test_velocity(0, 0, -M_PI, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(63), data.at(5));
}

TEST(Velocity_value, omega_2pi)
{
  std::string data;
  test_velocity(0, 0, 2*M_PI, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(254), data.at(5));
}

TEST(Velocity_value, omega_overflow)
{
  std::string data;
  test_velocity(0, 0, 3*M_PI, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(254), data.at(5));
}

TEST(Velocity_value, omega_underflow)
{
  std::string data;
  test_velocity(0, 0, -3*M_PI, &data);

  // vel_norm, vel_theta, omega
  EXPECT_EQ(char(0), data.at(3));
  EXPECT_EQ(char(0), data.at(4));
  EXPECT_EQ(char(0), data.at(5));
}

// =======================
//====== Command test =====
// =======================

TEST(Command, dribble_normal)
{
  RobotCommand  cmd(0, 0, 0, 0, 5, 0, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x80), data.at(6));
  EXPECT_EQ(char(0x50), data.at(7));
}

TEST(Command, dribble_overflow)
{
  RobotCommand  cmd(0, 0, 0, 0, 30, 0, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x80), data.at(6));
  EXPECT_EQ(char(0xF0), data.at(7));
}

TEST(Command, dribble_underflow)
{
  RobotCommand  cmd(0, 0, 0, 0, -30, 0, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x00), data.at(6));
  EXPECT_EQ(char(0x00), data.at(7));
}

TEST(Command, dribble_upperboundary_in)
{
  RobotCommand  cmd(0, 0, 0, 0, 15, 0, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x80), data.at(6));
  EXPECT_EQ(char(0xF0), data.at(7));
}

TEST(Command, dribble_upperboundary_out)
{
  RobotCommand  cmd(0, 0, 0, 0, 16, 0, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x80), data.at(6));
  EXPECT_EQ(char(0xF0), data.at(7));
}

TEST(Command, dribble_lowerboundary_in)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, 0, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x00), data.at(6));
  EXPECT_EQ(char(0x00), data.at(7));
}

TEST(Command, dribble_lowerboundary_out)
{
  RobotCommand  cmd(0, 0, 0, 0, -1, 0, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x00), data.at(6));
  EXPECT_EQ(char(0x00), data.at(7));
}


TEST(Command, kicker_normal)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, 8, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x10), data.at(6));
  EXPECT_EQ(char(0x08), data.at(7));
}

TEST(Command, kicker_overflow)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, 30, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x10), data.at(6));
  EXPECT_EQ(char(0x0F), data.at(7));
}

TEST(Command, kicker_underflow)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, -30, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x00), data.at(6));
  EXPECT_EQ(char(0x00), data.at(7));
}

TEST(Command, kicker_upperboundary_in)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, 15, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x10), data.at(6));
  EXPECT_EQ(char(0x0F), data.at(7));
}

TEST(Command, kicker_upperboundary_out)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, 16, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x10), data.at(6));
  EXPECT_EQ(char(0x0F), data.at(7));
}

TEST(Command, kicker_lowerboundary_in)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, 0, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x00), data.at(6));
  EXPECT_EQ(char(0x00), data.at(7));
}

TEST(Command, kicker_lowerboundary_out)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, -1, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x00), data.at(6));
  EXPECT_EQ(char(0x00), data.at(7));
}

TEST(Command, kicker_type_straight)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, 8, RobotCommand::STRAIGHT);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x10), data.at(6));
  EXPECT_EQ(char(0x08), data.at(7));
}

TEST(Command, kicker_type_chip)
{
  RobotCommand  cmd(0, 0, 0, 0, 0, 8, RobotCommand::CHIP);
  RootsSerializer rs;
  std::string data;
  rs.serialize(cmd, &data);
  //Action of machine
  EXPECT_EQ(char(0x18), data.at(6));
  EXPECT_EQ(char(0x08), data.at(7));
}


////////////////////////////////
// Checksum test
/////////////////////////////////
TEST(CheckSum, temp)
{
  //TODO:implement
  EXPECT_EQ(1, 1);
}
