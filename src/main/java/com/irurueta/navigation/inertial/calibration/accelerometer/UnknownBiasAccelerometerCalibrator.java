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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.units.Acceleration;

/**
 * Interface for accelerometer calibrator where bias is unknown and needs to
 * be estimated.
 */
public interface UnknownBiasAccelerometerCalibrator {

    /**
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return array containing x,y,z components of estimated accelerometer biases.
     */
    double[] getEstimatedBiases();

    /**
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @param result instance where estimated accelerometer biases will be stored.
     * @return true if result instance was updated, false otherwise (when estimation
     * is not yet available).
     */
    boolean getEstimatedBiases(final double[] result);

    /**
     * Gets column matrix containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return column matrix containing x,y,z components of estimated accelerometer
     * biases
     */
    Matrix getEstimatedBiasesAsMatrix();

    /**
     * Gets column matrix containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    boolean getEstimatedBiasesAsMatrix(final Matrix result)
            throws WrongSizeException;

    /**
     * Gets x coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return x coordinate of estimated accelerometer bias or null if not available.
     */
    Double getEstimatedBiasFx();

    /**
     * Gets y coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return y coordinate of estimated accelerometer bias or null if not available.
     */
    Double getEstimatedBiasFy();

    /**
     * Gets z coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return z coordinate of estimated accelerometer bias or null if not available.
     */
    Double getEstimatedBiasFz();

    /**
     * Gets x coordinate of estimated accelerometer bias.
     *
     * @return x coordinate of estimated accelerometer bias or null if not available.
     */
    Acceleration getEstimatedBiasFxAsAcceleration();

    /**
     * Gets x coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    boolean getEstimatedBiasFxAsAcceleration(final Acceleration result);

    /**
     * Gets y coordinate of estimated accelerometer bias.
     *
     * @return y coordinate of estimated accelerometer bias or null if not available.
     */
    Acceleration getEstimatedBiasFyAsAcceleration();

    /**
     * Gets y coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    boolean getEstimatedBiasFyAsAcceleration(final Acceleration result);

    /**
     * Gets z coordinate of estimated accelerometer bias.
     *
     * @return z coordinate of estimated accelerometer bias or null if not available.
     */
    Acceleration getEstimatedBiasFzAsAcceleration();

    /**
     * Gets z coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    boolean getEstimatedBiasFzAsAcceleration(final Acceleration result);

    /**
     * Gets estimated accelerometer bias.
     *
     * @return estimated accelerometer bias or null if not available.
     */
    AccelerationTriad getEstimatedBiasAsTriad();

    /**
     * Gets estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated accelerometer bias is available and result was
     * modified, false otherwise.
     */
    boolean getEstimatedBiasAsTriad(final AccelerationTriad result);
}
