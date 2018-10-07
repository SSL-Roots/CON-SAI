#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <gtest/gtest.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include "serializer.hpp"



TEST(HeaderTest, header_value)
{
    RootsSerializer rs;
    RobotCommand  cmd(0, 0, 0, 0, 0, 0, RobotCommand::STRAIGHT);

    std::string data;
    rs.serialize(cmd, &data);

    // HEADER
    EXPECT_EQ(char(0xFF), data.at(0));
    EXPECT_EQ(char(0xC3), data.at(1));
}

TEST(IDTest, id_value)
{
    RootsSerializer rs;
    RobotCommand  cmd(3, 0, 0, 0, 0, 0, RobotCommand::STRAIGHT);

    std::string data;
    rs.serialize(cmd, &data);

    // HEADER
    EXPECT_EQ(char(0x03), data.at(2));
}

//====== Velocity test =====
void convertVelocity(float v_norm, float v_theta, float omega, std::string* data)
{
    RootsSerializer rs;
    RobotCommand  cmd(0, v_norm, v_theta, omega, 0, 0, RobotCommand::STRAIGHT);
    rs.serialize(cmd, data);
}

TEST(VelocityTest, all_zero)
{
    std::string data;
    convertVelocity(0, 0, 0, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

TEST(VelocityTest, 2mps)
{
    std::string data;
    convertVelocity(2, 0, 0, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(127), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

TEST(VelocityTest, norm_overflow)
{
    std::string data;
    convertVelocity(100, 0, 0, &data);

    EXPECT_EQ(char(255), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

TEST(VelocityTest, norm_underflow)
{
    std::string data;
    convertVelocity(-100, 0, 0, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

TEST(VelocityTest, theta_plus)
{
    std::string data;
    convertVelocity(0, M_PI/2, 0, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(45), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

TEST(VelocityTest, theta_minus)
{
    std::string data;
    convertVelocity(0, -M_PI/2, 0, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(135), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

TEST(VelocityTest, theta_2pi)
{
    std::string data;
    convertVelocity(0, 2*M_PI, 0, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

TEST(VelocityTest, theta_overflow)
{
    std::string data;
    convertVelocity(0, 3*M_PI, 0, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(90), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

TEST(VelocityTest, theta_underflow)
{
    std::string data;
    convertVelocity(0, -3*M_PI, 0, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(90), data.at(4));
    EXPECT_EQ(char(127), data.at(5));
}

// =======================
// ===== Omega Test ======
// =======================
TEST(VelocityTest, omega_plus)
{
    std::string data;
    convertVelocity(0, 0, M_PI, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(191), data.at(5));
}

TEST(VelocityTest, omega_minus)
{
    std::string data;
    convertVelocity(0, 0, -M_PI, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(63), data.at(5));
}

TEST(VelocityTest, omega_2pi)
{
    std::string data;
    convertVelocity(0, 0, 2*M_PI, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(254), data.at(5));
}

TEST(VelocityTest, omega_overflow)
{
    std::string data;
    convertVelocity(0, 0, 3*M_PI, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(254), data.at(5));
}

TEST(VelocityTest, omega_underflow)
{
    std::string data;
    convertVelocity(0, 0, -3*M_PI, &data);

    // vel_norm, vel_theta, omega
    EXPECT_EQ(char(0), data.at(3));
    EXPECT_EQ(char(0), data.at(4));
    EXPECT_EQ(char(0), data.at(5));
}

// =======================
//====== Command test =====
// =======================

TEST(CommandTest, dribble_normal)
{
    RobotCommand  cmd(0, 0, 0, 0, 5, 0, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0xA6), data.at(6));
    EXPECT_EQ(char(0x50), data.at(7));
}

TEST(CommandTest, dribble_overflow)
{
    RobotCommand  cmd(0, 0, 0, 0, 30, 0, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0xA6), data.at(6));
    EXPECT_EQ(char(0xF0), data.at(7));
}

TEST(CommandTest, dribble_underflow)
{
    RobotCommand  cmd(0, 0, 0, 0, -30, 0, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x26), data.at(6));
    EXPECT_EQ(char(0x00), data.at(7));
}

TEST(CommandTest, dribble_upperboundary_in)
{
    RobotCommand  cmd(0, 0, 0, 0, 15, 0, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0xA6), data.at(6));
    EXPECT_EQ(char(0xF0), data.at(7));
}

TEST(CommandTest, dribble_upperboundary_out)
{
    RobotCommand  cmd(0, 0, 0, 0, 16, 0, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0xA6), data.at(6));
    EXPECT_EQ(char(0xF0), data.at(7));
}

TEST(CommandTest, dribble_lowerboundary_in)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, 0, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x26), data.at(6));
    EXPECT_EQ(char(0x00), data.at(7));
}

TEST(CommandTest, dribble_lowerboundary_out)
{
    RobotCommand  cmd(0, 0, 0, 0, -1, 0, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x26), data.at(6));
    EXPECT_EQ(char(0x00), data.at(7));
}


TEST(CommandTest, kicker_normal)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, 8, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x36), data.at(6));
    EXPECT_EQ(char(0x08), data.at(7));
}

TEST(CommandTest, kicker_overflow)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, 30, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x36), data.at(6));
    EXPECT_EQ(char(0x0F), data.at(7));
}

TEST(CommandTest, kicker_underflow)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, -30, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x26), data.at(6));
    EXPECT_EQ(char(0x00), data.at(7));
}

TEST(CommandTest, kicker_upperboundary_in)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, 15, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x36), data.at(6));
    EXPECT_EQ(char(0x0F), data.at(7));
}

TEST(CommandTest, kicker_upperboundary_out)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, 16, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x36), data.at(6));
    EXPECT_EQ(char(0x0F), data.at(7));
}

TEST(CommandTest, kicker_lowerboundary_in)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, 0, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x26), data.at(6));
    EXPECT_EQ(char(0x00), data.at(7));
}

TEST(CommandTest, kicker_lowerboundary_out)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, -1, RobotCommand::STRAIGHT);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x26), data.at(6));
    EXPECT_EQ(char(0x00), data.at(7));
}

TEST(CommandTest, kicker_type_chip)
{
    RobotCommand  cmd(0, 0, 0, 0, 0, 8, RobotCommand::CHIP);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0x3E), data.at(6));
    EXPECT_EQ(char(0x08), data.at(7));
}

TEST(CommandTest, dribble_and_chip)
{
    RobotCommand  cmd(0, 0, 0, 0, 5, 8, RobotCommand::CHIP);
    RootsSerializer rs;
    std::string data;
    rs.serialize(cmd, &data);
    //Action of machine
    EXPECT_EQ(char(0xBE), data.at(6));
    EXPECT_EQ(char(0x58), data.at(7));
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
