/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.gnss;

/**
 * Listener defining events of GNSSKalmanFilteredEstimatorListener.
 */
public interface GNSSKalmanFilteredEstimatorListener {

    /**
     * Called when measurements update starts.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateStart(final GNSSKalmanFilteredEstimator estimator);

    /**
     * Called when measurements update ends.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateEnd(final GNSSKalmanFilteredEstimator estimator);

    /**
     * Called when Kalman filter propagation starts.
     *
     * @param estimator estimator raising the event.
     */
    void onPropagateStart(final GNSSKalmanFilteredEstimator estimator);

    /**
     * Called when Kalman filter propagation ends.
     *
     * @param estimator estimator raising the event.
     */
    void onPropagateEnd(final GNSSKalmanFilteredEstimator estimator);

    /**
     * Called when estimator is reset.
     *
     * @param estimator estimator raising the event.
     */
    void onReset(final GNSSKalmanFilteredEstimator estimator);
}
