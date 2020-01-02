package com.irurueta.navigation.inertial;

/**
 * Listener defining events of INSGNSSTightlyCoupledKalmanFilteredEstimatorListener.
 */
public interface INSGNSSTightlyCoupledKalmanFilteredEstimatorListener {

    /**
     * Called when GNSS measurements update starts.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateGNSSMeasurementsStart(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when GNSS measurements update ends.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateGNSSMeasurementsEnd(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when inertial INS measurements update starts.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateBodyKinematicsStart(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when inertial INS measurements update ends.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateBodyKinematicsEnd(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when Kalman filter propagation starts.
     *
     * @param estimator estimator raising the event.
     */
    void onPropagateStart(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when Kalman filter propagation ends.
     *
     * @param estimator estimator raising the event.
     */
    void onPropagateEnd(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when estimator is reset.
     *
     * @param estimator estimator raising the event.
     */
    void onReset(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);
}
