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
import com.irurueta.units.AngularSpeed;

/**
 * Interface for gyroscope calibrator where bias is known.
 */
public interface KnownBiasGyroscopeCalibrator {

    /**
     * Gets known x coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return x coordinate of gyroscope bias.
     */
    double getBiasX();

    /**
     * Sets known x coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasX(final double biasX) throws LockedException;

    /**
     * Gets known y coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return y coordinate of gyroscope bias.
     */
    double getBiasY();

    /**
     * Sets known y coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasY y coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasY(final double biasY) throws LockedException;

    /**
     * Gets known z coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return z coordinate of gyroscope bias.
     */
    double getBiasZ();

    /**
     * Sets known z coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasZ(final double biasZ) throws LockedException;

    /**
     * Gets known x coordinate of gyroscope bias.
     *
     * @return x coordinate of gyroscope bias.
     */
    AngularSpeed getBiasAngularSpeedX();

    /**
     * Gets known x coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    void getBiasAngularSpeedX(final AngularSpeed result);

    /**
     * Sets known x coordinate of gyroscope bias.
     *
     * @param biasX x coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasX(final AngularSpeed biasX) throws LockedException;

    /**
     * Gets known y coordinate of gyroscope bias.
     *
     * @return y coordinate of gyroscope bias.
     */
    AngularSpeed getBiasAngularSpeedY();

    /**
     * Gets known y coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    void getBiasAngularSpeedY(final AngularSpeed result);

    /**
     * Sets known y coordinate of gyroscope bias.
     *
     * @param biasY y coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasY(final AngularSpeed biasY) throws LockedException;

    /**
     * Gets known z coordinate of gyroscope bias.
     *
     * @return z coordinate of gyroscope bias.
     */
    AngularSpeed getBiasAngularSpeedZ();

    /**
     * Gets known z coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    void getBiasAngularSpeedZ(final AngularSpeed result);

    /**
     * Sets known z coordinate of gyroscope bias.
     *
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasZ(final AngularSpeed biasZ) throws LockedException;

    /**
     * Sets known gyroscope bias coordinates expressed in radians per second
     * (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
     * @param biasY y coordinate of gyroscope bias.
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasCoordinates(final double biasX, final double biasY, final double biasZ)
            throws LockedException;

    /**
     * Sets known gyroscope bias coordinates.
     *
     * @param biasX x coordinate of gyroscope bias.
     * @param biasY y coordinate of gyroscope bias.
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setBiasCoordinates(final AngularSpeed biasX, final AngularSpeed biasY,
                            final AngularSpeed biasZ) throws LockedException;

    /**
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinate of known bias.
     */
    double[] getBias();

    /**
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void getBias(final double[] result);

    /**
     * Sets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known gyroscope bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void setBias(final double[] bias) throws LockedException;

    /**
     * Gets known gyroscope bias as a column matrix.
     *
     * @return known gyroscope bias as a column matrix.
     */
    Matrix getBiasAsMatrix();

    /**
     * Gets known gyroscope bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void getBiasAsMatrix(final Matrix result);

    /**
     * Sets known gyroscope bias as a column matrix.
     *
     * @param bias gyroscope bias to be set.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void setBias(final Matrix bias) throws LockedException;
}
