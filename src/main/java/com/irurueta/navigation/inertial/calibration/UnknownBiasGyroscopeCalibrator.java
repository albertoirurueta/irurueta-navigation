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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.units.AngularSpeed;

/**
 * Interface for gyroscope calibrator where bias is unknown and needs to
 * be estimated.
 */
public interface UnknownBiasGyroscopeCalibrator {

    /**
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return array containing x,y,z components of estimated gyroscope biases.
     */
    double[] getEstimatedBiases();

    /**
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @param result instance where estimated gyroscope biases will be stored.
     * @return true if result instance was updated, false otherwise (when estimation
     * is not yet available).
     */
    boolean getEstimatedBiases(final double[] result);

    /**
     * Gets column matrix containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return column matrix containing x,y,z component of estimated gyroscope
     * biases.
     */
    Matrix getEstimatedBiasesAsMatrix();

    /**
     * Gets column matrix containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    boolean getEstimatedBiasesAsMatrix(final Matrix result)
            throws WrongSizeException;

    /**
     * Gets x coordinate of estimated gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    Double getEstimatedBiasX();

    /**
     * Gets y coordinate of estimated gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    Double getEstimatedBiasY();

    /**
     * Gets z coordinate of estimated gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return z coordinate of estimated gyroscope bias or null if not available.
     */
    Double getEstimatedBiasZ();

    /**
     * Gets x coordinate of estimated gyroscope bias.
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    AngularSpeed getEstimatedBiasAngularSpeedX();

    /**
     * Gets x coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    boolean getEstimatedBiasAngularSpeedX(final AngularSpeed result);

    /**
     * Gets y coordinate of estimated gyroscope bias.
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    AngularSpeed getEstimatedBiasAngularSpeedY();

    /**
     * Gets y coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    boolean getEstimatedBiasAngularSpeedY(final AngularSpeed result);

    /**
     * Gets z coordinate of estimated gyroscope bias.
     *
     * @return z coordinate of estimated gyroscope bias or null if not available.
     */
    AngularSpeed getEstimatedBiasAngularSpeedZ();

    /**
     * Gets z coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    boolean getEstimatedBiasAngularSpeedZ(final AngularSpeed result);
}
