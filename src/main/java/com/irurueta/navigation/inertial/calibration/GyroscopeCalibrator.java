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
import com.irurueta.navigation.NotReadyException;

/**
 * Interface for gyroscope calibrators.
 */
public interface GyroscopeCalibrator {
    /**
     * Indicates whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     *
     * @return true if z-axis is assumed to be common for accelerometer and gyroscope,
     * false otherwise.
     */
    boolean isCommonAxisUsed();

    /**
     * Specifies whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     *
     * @param commonAxisUsed true if z-axis is assumed to be common for accelerometer
     *                       and gyroscope, false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    void setCommonAxisUsed(final boolean commonAxisUsed) throws LockedException;

    /**
     * Indicates whether calibrator is ready to start the calibration.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    boolean isReady();

    /**
     * Indicates whether calibrator is currently running or not.
     *
     * @return true if calibrator is running, false otherwise.
     */
    boolean isRunning();

    /**
     * Estimates gyroscope calibration parameters containing bias, scale factors,
     * cross-coupling errors and g-dependant cross biases.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if calibration fails for numerical reasons.
     */
    void calibrate() throws LockedException, NotReadyException, CalibrationException;

    /**
     * Gets estimated gyroscope scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated gyroscope scale factors and cross coupling errors.
     */
    Matrix getEstimatedMg();

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    Double getEstimatedSx();

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    Double getEstimatedSy();

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    Double getEstimatedSz();

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    Double getEstimatedMxy();

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    Double getEstimatedMxz();

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    Double getEstimatedMyx();

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    Double getEstimatedMyz();

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    Double getEstimatedMzx();

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    Double getEstimatedMzy();

    /**
     * Gets estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @return a 3x3 matrix containing g-dependent cross biases.
     */
    Matrix getEstimatedGg();
}
