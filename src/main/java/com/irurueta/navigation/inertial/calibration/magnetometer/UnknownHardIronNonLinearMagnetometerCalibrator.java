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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.units.MagneticFluxDensity;

/**
 * Interface for non-linear magnetometer calibrator where hard-iron bias is
 * unknown and needs to be estimated.
 */
public interface UnknownHardIronNonLinearMagnetometerCalibrator
        extends UnknownHardIronMagnetometerCalibrator {

    /**
     * Gets initial x-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial x-coordinate of magnetometer hard-iron bias.
     */
    double getInitialHardIronX();

    /**
     * Sets initial x-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIronX(final double initialHardIronX)
            throws LockedException;

    /**
     * Gets initial y-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial y-coordinate of magnetometer hard-iron bias.
     */
    double getInitialHardIronY();

    /**
     * Sets initial y-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIronY(final double initialHardIronY)
            throws LockedException;

    /**
     * Gets initial z-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial z-coordinate of magnetometer hard-iron bias.
     */
    double getInitialHardIronZ();

    /**
     * Sets initial z-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in meters Teslas (T).
     *
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIronZ(final double initialHardIronZ)
            throws LockedException;

    /**
     * Gets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial x-coordinate of magnetometer hard-iron bias.
     */
    MagneticFluxDensity getInitialHardIronXAsMagneticFluxDensity();

    /**
     * Gets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    void getInitialHardIronXAsMagneticFluxDensity(
            final MagneticFluxDensity result);

    /**
     * Sets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIronX(final MagneticFluxDensity initialHardIronX)
            throws LockedException;

    /**
     * Gets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial y-coordinate of magnetometer hard-iron bias.
     */
    MagneticFluxDensity getInitialHardIronYAsMagneticFluxDensity();

    /**
     * Gets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    void getInitialHardIronYAsMagneticFluxDensity(
            final MagneticFluxDensity result);

    /**
     * Sets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronY initial y-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIronY(final MagneticFluxDensity initialHardIronY)
            throws LockedException;

    /**
     * Gets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial z-coordinate of magnetometer hard-iron bias.
     */
    MagneticFluxDensity getInitialHardIronZAsMagneticFluxDensity();

    /**
     * Gets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    void getInitialHardIronZAsMagneticFluxDensity(
            final MagneticFluxDensity result);

    /**
     * Sets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronZ initial z-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIronZ(final MagneticFluxDensity initialHardIronZ)
            throws LockedException;

    /**
     * Sets initial hard-iron bias coordinates of magnetometer used to find
     * a solution expressed in Teslas (T).
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias.
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias.
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIron(
            final double initialHardIronX,
            final double initialHardIronY,
            final double initialHardIronZ) throws LockedException;

    /**
     * Sets initial hard iron coordinates of magnetometer used to find a solution.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer bias.
     * @param initialHardIronY initial y-coordinate of magnetometer bias.
     * @param initialHardIronZ initial z-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIron(
            final MagneticFluxDensity initialHardIronX,
            final MagneticFluxDensity initialHardIronY,
            final MagneticFluxDensity initialHardIronZ) throws LockedException;

    /**
     * Gets initial hard-iron used to find a solution.
     *
     * @return initial hard-iron.
     */
    MagneticFluxDensityTriad getInitialHardIronAsTriad();

    /**
     * Gets initial hard-iron used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    void getInitialHardIronAsTriad(final MagneticFluxDensityTriad result);

    /**
     * Sets initial hard-iron used to find a solution.
     *
     * @param initialHardIron initial hard-iron to be set.
     * @throws LockedException if calibrator is currently running.
     */
    void setInitialHardIron(final MagneticFluxDensityTriad initialHardIron)
            throws LockedException;

    /**
     * Gets initial hard-iron bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @return array containing coordinates of initial bias.
     */
    double[] getInitialHardIron();

    /**
     * Gets initial hard-iron  bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    void getInitialHardIron(final double[] result);

    /**
     * Sets initial hard-iron bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void setInitialHardIron(final double[] initialHardIron)
            throws LockedException;

    /**
     * Gets initial hard-iron bias to be used to find a solution as a
     * column matrix.
     *
     * @return initial hard-iron bias to be used to find a solution as a
     * column matrix.
     */
    Matrix getInitialHardIronAsMatrix();

    /**
     * Gets initial hard-iron bias to be used to find a solution as a
     * column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void getInitialHardIronAsMatrix(final Matrix result);

    /**
     * Sets initial hard-iron bias to be used to find a solution as an array.
     *
     * @param initialHardIron initial hard-iron bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void setInitialHardIron(final Matrix initialHardIron)
            throws LockedException;
}
