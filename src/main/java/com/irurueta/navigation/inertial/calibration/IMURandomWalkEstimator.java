package com.irurueta.navigation.inertial.calibration;

public class IMURandomWalkEstimator {

    // TODO: to compute position and velocity accuracy, use ECEFInertialNavigator to accumulate all kinematics samples
    //  and update current frame to determine changes in position and velocity when it should remain static --> this
    //  can be used to estimate positionNoiseSD and velocityNoiseSD fro INSLooseCoupledKalmanConfig

    // TODO: INSLooselyCoupledKalmanConfig requires gyro and accelerometer noise PSD. It can be obtained from
    //  generators / interval detector / noise estimator

    // DONE

    // TODO: accumulate multiple estimations with BodyKinematicsBiasEstimator to account for drifts on estimated accelerometer
    //  and gyroscope biases (accelerometer and gyro random walks)

    // TODO: INSLooselyCoupledKalmanConfig requires accelerometer and gyro bias PSD, which is the PSD of the bias
    //  variation with time (PSD of bias / time). We could modify BodyKinematicsBiasEstimator to also estimate variation
    //  of bias through time

    // TODO: INSLooselyCoupledKalmanInitializerConfig requires accelerometer and gyroscope bias uncertainty. We need
    //  methods from calibrators to obtain bias uncertainty from the estimated covariance matrix
}
