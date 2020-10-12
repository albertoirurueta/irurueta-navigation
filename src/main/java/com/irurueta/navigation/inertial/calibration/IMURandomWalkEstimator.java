package com.irurueta.navigation.inertial.calibration;

public class IMURandomWalkEstimator {

    // TODO: to compute position and velocity accuracy, use ECEFInertialNavigator to accumulate all kinematics samples
    //  and update current frame to determine changes in position and velocity when it should remain static

    // TODO: accumulate multiple estimations with BodyKinematicsBiasEstimator to account for drifts on estimated accelerometer
    //  and gyroscope biases (accelerometer and gyro random walks)
}
