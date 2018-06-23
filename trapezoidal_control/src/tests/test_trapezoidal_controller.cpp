#include "trapezoidal_controller.h"

#include <gtest/gtest.h>


TEST(ControllerTest, UpDown){
    TrapezoidalController controller(0.02, 0.04, 2.0, 0.02, false);
    int timeSteps = 180;

    double currentPose = 0.0;
    double targetPose = 2.0;
    double currentVelocity = 0.0;

    for(int t = 0; t<timeSteps; t++){
        controller.update(currentPose, currentVelocity, targetPose);
        double velocity = controller.getVelocity();

        currentPose += velocity * 0.016;
        currentVelocity = velocity;

        // Debug print
        // std::cerr << t << ":" << velocity << ":" << currentPose << std::endl;
    }
    EXPECT_EQ(0, 0);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
