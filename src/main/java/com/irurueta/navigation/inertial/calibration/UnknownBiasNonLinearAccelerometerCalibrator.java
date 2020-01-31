/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.units.Acceleration;

/**
 * Interface for non-linear accelerometer calibrator where bias is unknown
 * and needs to be estimated.
 */
public interface UnknownBiasNonLinearAccelerometerCalibrator
        extends UnknownBiasAccelerometerCalibrator {

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return initial x-coordinate of accelerometer bias.
     */
    double getInitialBiasX();

    /**
     * Sets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasX(final double initialBiasX) throws LockedException;

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return initial y-coordinate of accelerometer bias.
     */
    double getInitialBiasY();

    /**
     * Sets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasY(final double initialBiasY) throws LockedException;

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return initial z-coordinate of accelerometer bias.
     */
    double getInitialBiasZ();

    /**
     * Sets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasZ(final double initialBiasZ) throws LockedException;

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
     *
     * @return initial x-coordinate of accelerometer bias.
     */
    Acceleration getInitialBiasXAsAcceleration();

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    void getInitialBiasXAsAcceleration(final Acceleration result);

    /**
     * Sets initial x-coordinate of accelerometer bias to be used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasX(final Acceleration initialBiasX)
            throws LockedException;

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     *
     * @return initial y-coordinate of accelerometer bias.
     */
    Acceleration getInitialBiasYAsAcceleration();

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    void getInitialBiasYAsAcceleration(final Acceleration result);

    /**
     * Sets initial y-coordinate of accelerometer bias to be used to find a solution.
     *
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasY(final Acceleration initialBiasY)
            throws LockedException;

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     *
     * @return initial z-coordinate of accelerometer bias.
     */
    Acceleration getInitialBiasZAsAcceleration();

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    void getInitialBiasZAsAcceleration(final Acceleration result);

    /**
     * Sets initial z-coordinate of accelerometer bias to be used to find a solution.
     *
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBiasZ(final Acceleration initialBiasZ)
            throws LockedException;

    /**
     * Sets initial bias coordinates of accelerometer used to find a solution
     * expressed in meters per squared second (m/s^2).
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBias(final double initialBiasX, final double initialBiasY,
                        final double initialBiasZ) throws LockedException;

    /**
     * Sets initial bias coordinates of accelerometer used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialBias(final Acceleration initialBiasX,
                        final Acceleration initialBiasY,
                        final Acceleration initialBiasZ) throws LockedException;

    /**
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @return array containing coordinates of initial bias.
     */
    double[] getInitialBias();

    /**
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void getInitialBias(final double[] result);

    /**
     * Sets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void setInitialBias(final double[] initialBias) throws LockedException;

    /**
     * Gets initial bias to be used to find a solution as a column matrix.
     *
     * @return initial bias to be used to find a solution as a column matrix.
     */
    Matrix getInitialBiasAsMatrix();

    /**
     * Gets initial bias to be used to find a solution as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void getInitialBiasAsMatrix(final Matrix result);

    /**
     * Sets initial bias to be used to find a solution as an array.
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void setInitialBias(final Matrix initialBias) throws LockedException;
}
