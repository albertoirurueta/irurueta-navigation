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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

/**
 * Initializes the tightly coupled INS/GNSS extended Kalman filter error covariance
 * matrix.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/Initialize_TC_P_matrix.m
 */
public class INSTightlyCoupledKalmanInitializer {

    /**
     * Number of parameters of the Kalman filter.
     */
    public static final int NUM_PARAMS = 17;

    /**
     * Initializes INS/GNS tightly coupled Kalman filter error covariance matrix.
     *
     * @param config Kalman filter configuration.
     * @param result instance where resulting initialized error covariance matrix
     *               will be stored. Matrix must be 17x17, otherwise it will be resized.
     */
    public static void initialize(final INSTightlyCoupledKalmanInitializerConfig config,
                                  final Matrix result) {
        if (result.getRows() != NUM_PARAMS || result.getColumns() != NUM_PARAMS) {
            try {
                result.resize(NUM_PARAMS, NUM_PARAMS);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        final double initAttUnc = config.getInitialAttitudeUncertainty();
        final double initVelUnc = config.getInitialVelocityUncertainty();
        final double initPosUnc = config.getInitialPositionUncertainty();
        final double initBaUnc = config.getInitialAccelerationBiasUncertainty();
        final double initBgUnc = config.getInitialGyroscopeBiasUncertainty();
        final double initClockOffset = config.getInitialClockOffsetUncertainty();
        final double initClockDrift = config.getInitialClockDriftUncertainty();

        final double initAttUnc2 = initAttUnc * initAttUnc;
        final double initVelUnc2 = initVelUnc * initVelUnc;
        final double initPosUnc2 = initPosUnc * initPosUnc;
        final double initBaUnc2 = initBaUnc * initBaUnc;
        final double initBgUnc2 = initBgUnc * initBgUnc;
        final double initClockOffset2 = initClockOffset * initClockOffset;
        final double initClockDrift2 = initClockDrift * initClockDrift;

        result.initialize(0.0);

        for (int i = 0; i < 3; i++) {
            result.setElementAt(i, i, initAttUnc2);
        }
        for (int i = 3; i < 6; i++) {
            result.setElementAt(i, i, initVelUnc2);
        }
        for (int i = 6; i < 9; i++) {
            result.setElementAt(i, i, initPosUnc2);
        }
        for (int i = 9; i < 12; i++) {
            result.setElementAt(i, i, initBaUnc2);
        }
        for (int i = 12; i < 15; i++) {
            result.setElementAt(i, i, initBgUnc2);
        }
        result.setElementAt(15, 15, initClockOffset2);
        result.setElementAt(16, 16, initClockDrift2);
    }

    /**
     * Initializes INS/GNS tightly coupled Kalman filter error covariance matrix.
     *
     * @param config Kalman filter configuration.
     * @return initialized error covariance matrix.
     */
    public static Matrix initialize(final INSTightlyCoupledKalmanInitializerConfig config) {
        Matrix result = null;
        try {
            result = new Matrix(NUM_PARAMS, NUM_PARAMS);
            initialize(config, result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        return result;
    }
}
