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
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.units.Acceleration;

import java.util.Collection;

/**
 * Estimates accelerometer biases, cross couplings and scaling factors.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single position where
 * gravity norm is known must be taken at 10 different unknown orientations and zero
 * velocity when common z-axis is assumed, otherwise at least 13 measurements are required.
 * <p>
 * Measured specific force is assumed to follow the model shown below:
 * <pre>
 *     fmeas = ba + (I + Ma) * ftrue + w
 * </pre>
 * Where:
 * - fmeas is the measured specific force. This is a 3x1 vector.
 * - ba is accelerometer bias. Ideally, on a perfect accelerometer, this should be a
 * 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Ma is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect accelerometer, this should be a 3x3 zero matrix.
 * - ftrue is ground-trush specific force.
 * - w is measurement noise.
 */
public class KnownGravityNormAccelerometerCalibrator extends
        BaseGravityNormAccelerometerCalibrator<KnownGravityNormAccelerometerCalibrator,
                KnownGravityNormAccelerometerCalibratorListener> {

    /**
     * Constructor.
     */
    public KnownGravityNormAccelerometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     */
    public KnownGravityNormAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        super(initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        super(initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy,
                initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        super(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final double[] initialBias) {
        super(initialBias);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        super(measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] initialBias) {
        super(commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownGravityNormAccelerometerCalibrator(final Matrix initialBias) {
        super(initialBias);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        super(measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Matrix initialBias, final Matrix initialMa) {
        super(initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Matrix initialBias, final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(measurements, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        super(commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        super(measurements, commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm) {
        super(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements) {
        super(groundTruthGravityNorm, measurements);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed) {
        super(groundTruthGravityNorm, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBiasX, initialBiasY,
                initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        super(groundTruthGravityNorm, measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        super(groundTruthGravityNorm, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double[] initialBias) {
        super(groundTruthGravityNorm, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double[] initialBias) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix initialBias) {
        super(groundTruthGravityNorm, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix initialBias, final Matrix initialMa) {
        super(groundTruthGravityNorm, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix initialBias, final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(final Acceleration groundTruthGravityNorm) {
        this(convertAcceleration(groundTruthGravityNorm));
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements) {
        this(convertAcceleration(groundTruthGravityNorm), measurements);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final boolean commonAxisUsed) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final boolean commonAxisUsed,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(convertAcceleration(groundTruthGravityNorm), initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(convertAcceleration(groundTruthGravityNorm), initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final double[] initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final boolean commonAxisUsed,
            final double[] initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final boolean commonAxisUsed,
            final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Matrix initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Matrix initialBias, final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm), initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Matrix initialBias, final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias, initialMa,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed,
                initialBias, initialMa, listener);
    }

    /**
     * Sets ground truth gravity norm to be expected at location where
     * measurements have been made, expressed in meters per squared second
     * (m/s^2).
     *
     * @param groundTruthGravityNorm ground truth gravity norm or
     *                               null if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setGroundTruthGravityNorm(final Double groundTruthGravityNorm)
            throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Sets ground truth gravity norm to be expected at location where
     * measurements have been made.
     *
     * @param groundTruthGravityNorm ground truth gravity norm or null
     *                               if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setGroundTruthGravityNorm(
            final Acceleration groundTruthGravityNorm)
            throws LockedException {
        setGroundTruthGravityNorm(groundTruthGravityNorm != null ?
                convertAcceleration(groundTruthGravityNorm) : null);
    }
}
