#include "trapezoidal_controller.h"

#include <gtest/gtest.h>

TEST(InitializeTest, Velocity){
    TrapezoidalController controller;
    double velocity = controller.getVelocity();
    EXPECT_EQ(velocity, 0);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
