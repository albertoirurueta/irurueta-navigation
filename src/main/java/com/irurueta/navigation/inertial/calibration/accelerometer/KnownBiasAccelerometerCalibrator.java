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
package com.irurueta.navigation.inertial.calibration.accelerometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.units.Acceleration;

/**
 * Interface for accelerometer calibrator where bias is known.
 */
public interface KnownBiasAccelerometerCalibrator {

    /**
     * Gets known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return x coordinate of accelerometer bias.
     */
    double getBiasX();

    /**
     * Sets known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasX(final double biasX) throws LockedException;

    /**
     * Gets known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return y coordinate of accelerometer bias.
     */
    double getBiasY();

    /**
     * Sets known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasY y coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasY(final double biasY) throws LockedException;

    /**
     * Gets known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return z coordinate of accelerometer bias.
     */
    double getBiasZ();

    /**
     * Sets known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasZ(final double biasZ) throws LockedException;

    /**
     * Gets known x coordinate of accelerometer bias.
     *
     * @return x coordinate of accelerometer bias.
     */
    Acceleration getBiasXAsAcceleration();

    /**
     * Gets known x coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    void getBiasXAsAcceleration(final Acceleration result);

    /**
     * Sets known x coordinate of accelerometer bias.
     *
     * @param biasX x coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasX(final Acceleration biasX) throws LockedException;

    /**
     * Gets known y coordinate of accelerometer bias.
     *
     * @return y coordinate of accelerometer bias.
     */
    Acceleration getBiasYAsAcceleration();

    /**
     * Gets known y coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    void getBiasYAsAcceleration(final Acceleration result);

    /**
     * Sets known y coordinate of accelerometer bias.
     *
     * @param biasY y coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    void setBiasY(final Acceleration biasY) throws LockedException;

    /**
     * Gets known z coordinate of accelerometer bias.
     *
     * @return z coordinate of accelerometer bias.
     */
    Acceleration getBiasZAsAcceleration();

    /**
     * Gets known z coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    void getBiasZAsAcceleration(final Acceleration result);

    /**
     * Sets known z coordinate of accelerometer bias.
     *
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    void setBiasZ(final Acceleration biasZ) throws LockedException;

    /**
     * Sets known accelerometer bias coordinates expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x coordinate of accelerometer bias.
     * @param biasY y coordinate of accelerometer bias.
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasCoordinates(final double biasX, final double biasY, final double biasZ)
            throws LockedException;

    /**
     * Sets known accelerometer bias coordinates.
     *
     * @param biasX z coordinate of accelerometer bias.
     * @param biasY y coordinate of accelerometer bias.
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasCoordinates(final Acceleration biasX, final Acceleration biasY,
                            final Acceleration biasZ) throws LockedException;

    /**
     * Gets known accelerometer bias.
     *
     * @return known accelerometer bias.
     */
    AccelerationTriad getBiasAsTriad();

    /**
     * Gets known accelerometer bias.
     *
     * @param result instance where result will be stored.
     */
    void getBiasAsTriad(final AccelerationTriad result);

    /**
     * Sets known accelerometer bias.
     *
     * @param bias accelerometer bias to be set.
     * @throws LockedException if calibrator is currently running.
     */
    void setBias(final AccelerationTriad bias) throws LockedException;

    /**
     * Gets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @return array containing coordinates of known bias.
     */
    double[] getBias();

    /**
     * Gets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void getBias(final double[] result);

    /**
     * Sets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param bias known accelerometer bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void setBias(final double[] bias) throws LockedException;

    /**
     * Gets known accelerometer bias as a column matrix.
     *
     * @return known accelerometer bias as a column matrix.
     */
    Matrix getBiasAsMatrix();

    /**
     * Gets known accelerometer bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void getBiasAsMatrix(final Matrix result);

    /**
     * Sets known accelerometer bias as a column matrix.
     *
     * @param bias accelerometer bias to be set.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void setBias(final Matrix bias) throws LockedException;
}
