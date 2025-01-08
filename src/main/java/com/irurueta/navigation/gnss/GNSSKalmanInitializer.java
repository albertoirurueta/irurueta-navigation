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
 * Initializes the GNSS EKF Kalman filter state estimates and error covariance matrix.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/Initialize_GNSS_KF.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/Initialize_GNSS_KF.m
 * </a>
 */
public class GNSSKalmanInitializer {

    /**
     * Constructor.
     * Prevents instantiation of utility class.
     */
    private GNSSKalmanInitializer() {
    }

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

        final var initPosUnc = config.getInitialPositionUncertainty();
        final var initVelUnc = config.getInitialVelocityUncertainty();
        final var initClockOffsetUnc = config.getInitialClockOffsetUncertainty();
        final var initClockDriftUnc = config.getInitialClockDriftUncertainty();

        final var initPosUnc2 = initPosUnc * initPosUnc;
        final var initVelUnc2 = initVelUnc * initVelUnc;
        final var initClockOffsetUnc2 = initClockOffsetUnc * initClockOffsetUnc;
        final var initClockDriftUnc2 = initClockDriftUnc * initClockDriftUnc;

        Matrix covariance;
        try {
            covariance = new Matrix(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);

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
    public static GNSSKalmanState initialize(final GNSSEstimation estimation, final GNSSKalmanConfig config) {
        final var result = new GNSSKalmanState();
        initialize(estimation, config, result);
        return result;
    }
}
