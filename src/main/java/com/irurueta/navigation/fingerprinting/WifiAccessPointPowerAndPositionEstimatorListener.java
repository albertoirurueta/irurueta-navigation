package com.irurueta.navigation.fingerprinting;

import com.irurueta.geometry.Point;

/**
 * Listener defining events of WifiAccessPointPowerAndPositionEstimator
 * @param <P> a {@link Point} type.
 */
public interface WifiAccessPointPowerAndPositionEstimatorListener<P extends Point> {

    /**
     * Called when estimation starts.
     * @param estimator estimator raising the event.
     */
    void onEstimateStart(WifiAccessPointPowerAndPositionEstimator<P> estimator);

    /**
     * Called when estimation ends.
     * @param estimator estimator raising the event.
     */
    void onEstimateEnd(WifiAccessPointPowerAndPositionEstimator<P> estimator);
}
