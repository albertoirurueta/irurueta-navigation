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
 * Interface for magnetometer calibrator where hard-iron is known.
 * Hard-iron term of magnetometer model behaves like accelerometer bias.
 */
public interface KnownHardIronMagnetometerCalibrator {

    /**
     * Gets known x coordinate of magnetometer hard-iron expressed in
     * Teslas (T).
     *
     * @return x coordinate of magnetometer hard-iron.
     */
    double getHardIronX();

    /**
     * Sets known x coordinate of magnetometer hard-iron expressed in
     * Teslas (T).
     *
     * @param hardIronX x coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIronX(final double hardIronX) throws LockedException;

    /**
     * Gets known y coordinate of magnetometer hard-iron expressed in
     * Teslas (T).
     *
     * @return y coordinate of magnetometer hard-iron.
     */
    double getHardIronY();

    /**
     * Sets known y coordinate of magnetometer hard-iron expressed in
     * Teslas (T).
     *
     * @param hardIronY y coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIronY(final double hardIronY) throws LockedException;

    /**
     * Gets known z coordinate of magnetometer hard-iron expressed in
     * Teslas (T).
     *
     * @return z coordinate of magnetometer hard-iron.
     */
    double getHardIronZ();

    /**
     * Sets known z coordinate of magnetometer hard-iron expressed in
     * Teslas (T).
     *
     * @param hardIronZ z coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIronZ(final double hardIronZ) throws LockedException;

    /**
     * Gets known x coordinate of magnetometer hard-iron.
     *
     * @return x coordinate of magnetometer hard-iron.
     */
    MagneticFluxDensity getHardIronXAsMagneticFluxDensity();

    /**
     * Gets known x coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    void getHardIronXAsMagneticFluxDensity(final MagneticFluxDensity result);

    /**
     * Sets known x-coordinate of magnetometer hard-iron.
     *
     * @param hardIronX known x-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIronX(final MagneticFluxDensity hardIronX) throws LockedException;

    /**
     * Gets known y coordinate of magnetometer hard-iron.
     *
     * @return y coordinate of magnetometer hard-iron.
     */
    MagneticFluxDensity getHardIronYAsMagneticFluxDensity();

    /**
     * Gets known y coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    void getHardIronYAsMagneticFluxDensity(final MagneticFluxDensity result);

    /**
     * Sets known y-coordinate of magnetometer hard-iron.
     *
     * @param hardIronY known y-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIronY(final MagneticFluxDensity hardIronY) throws LockedException;

    /**
     * Gets known z coordinate of magnetometer hard-iron.
     *
     * @return z coordinate of magnetometer hard-iron.
     */
    MagneticFluxDensity getHardIronZAsMagneticFluxDensity();

    /**
     * Gets known z coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    void getHardIronZAsMagneticFluxDensity(final MagneticFluxDensity result);

    /**
     * Sets known z-coordinate of magnetometer hard-iron.
     *
     * @param hardIronZ known z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIronZ(final MagneticFluxDensity hardIronZ) throws LockedException;

    /**
     * Sets known hard-iron coordinates expressed in Teslas (T).
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron.
     * @param hardIronY y-coordinate of magnetometer hard-iron.
     * @param hardIronZ z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIronCoordinates(
            final double hardIronX, final double hardIronY,
            final double hardIronZ) throws LockedException;

    /**
     * Sets known hard-iron coordinates.
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron.
     * @param hardIronY y-coordinate of magnetometer hard-iron.
     * @param hardIronZ z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIronCoordinates(
            final MagneticFluxDensity hardIronX,
            final MagneticFluxDensity hardIronY,
            final MagneticFluxDensity hardIronZ) throws LockedException;

    /**
     * Gets known hard-iron.
     *
     * @return known hard-iron.
     */
    MagneticFluxDensityTriad getHardIronAsTriad();

    /**
     * Gets known hard-iron.
     *
     * @param result instance where result will be stored.
     */
    void getHardIronAsTriad(final MagneticFluxDensityTriad result);

    /**
     * Sets known hard-iron.
     *
     * @param hardIron hard-iron to be set.
     * @throws LockedException if calibrator is currently running.
     */
    void setHardIron(final MagneticFluxDensityTriad hardIron)
            throws LockedException;

    /**
     * Gets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @return array containing coordinates of known hard-iron bias.
     */
    double[] getHardIron();

    /**
     * Gets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void getHardIron(final double[] result);

    /**
     * Sets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param hardIron known hard-iron bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    void setHardIron(final double[] hardIron) throws LockedException;

    /**
     * Gets known hard-iron bias as a column matrix.
     *
     * @return known hard-iron bias as a column matrix.
     */
    Matrix getHardIronMatrix();

    /**
     * Gets known hard-iron bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void getHardIronMatrix(final Matrix result);

    /**
     * Sets known hard-iron bias as a column matrix.
     *
     * @param hardIron magnetometer hard-iron bias to be set.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    void setHardIron(final Matrix hardIron) throws LockedException;
}
