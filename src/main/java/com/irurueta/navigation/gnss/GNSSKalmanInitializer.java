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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

/**
 * Initializes GNSS Kalman filter state.
 */
public class GNSSKalmanInitializer {

    /**
     * Initializes GNSS Kalman filter state.
     *
     * @param estimation initial GNSS estimation containing ECEF position and velocity,
     *                   and estimated clock offset and drift.
     * @param config     Kalman filter configuration.
     * @param result     instance where resulting initialized Kalman filter state will be
     *                   stored.
     */
    public static void initialize(final GNSSEstimation estimation,
                                  final GNSSKalmanConfig config,
                                  final GNSSKalmanState result) {
        result.setEstimation(estimation);

        final double initPosUnc = config.getInitialPositionUncertainty();
        final double initVelUnc = config.getInitialVelocityUncertainty();
        final double initClockOffsetUnc = config.getInitialClockOffsetUncertainty();
        final double initClockDriftUnc = config.getInitialClockDriftUncertainty();

        final double initPosUnc2 = initPosUnc * initPosUnc;
        final double initVelUnc2 = initVelUnc * initVelUnc;
        final double initClockOffsetUnc2 = initClockOffsetUnc * initClockOffsetUnc;
        final double initClockDriftUnc2 = initClockDriftUnc * initClockDriftUnc;

        Matrix covariance;
        try {
            covariance = new Matrix(GNSSEstimation.NUM_PARAMETERS,
                    GNSSEstimation.NUM_PARAMETERS);

            covariance.setElementAt(0, 0, initPosUnc2);
            covariance.setElementAt(1, 1, initPosUnc2);
            covariance.setElementAt(2, 2, initPosUnc2);

            covariance.setElementAt(3, 3, initVelUnc2);
            covariance.setElementAt(4, 4, initVelUnc2);
            covariance.setElementAt(5, 5, initVelUnc2);

            covariance.setElementAt(6, 6, initClockOffsetUnc2);
            covariance.setElementAt(7, 7, initClockDriftUnc2);

            result.setCovariance(covariance);

        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Initializes GNSS Kalman filter state.
     *
     * @param estimation initial GNSS estimation containing ECEF position and velocity,
     *                   and estimated clock offset and drift.
     * @param config     Kalman filter configuration.
     * @return initialized Kalman filter state.
     */
    public static GNSSKalmanState initialize(final GNSSEstimation estimation,
                                             final GNSSKalmanConfig config) {
        final GNSSKalmanState result = new GNSSKalmanState();
        initialize(estimation, config, result);
        return result;
    }
}
