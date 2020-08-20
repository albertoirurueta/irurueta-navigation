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
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.units.Acceleration;

import java.util.Collection;

/**
 * Estimates accelerometer biases, cross couplings and scaling factors.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single known position must
 * be taken at 10 different unknown orientations and zero velocity when common z-axis
 * is assumed, otherwise at least 13 measurements are required.
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
public class KnownPositionAccelerometerCalibrator extends
        BaseGravityNormAccelerometerCalibrator<KnownPositionAccelerometerCalibrator,
                        KnownPositionAccelerometerCalibratorListener> {


    /**
     * Position where body kinematics measures have been taken.
     */
    private ECEFPosition mPosition;

    /**
     * Constructor.
     */
    public KnownPositionAccelerometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final KnownPositionAccelerometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownPositionAccelerometerCalibratorListener listener) {
        super(measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(final Matrix initialBias) {
        super(initialBias);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
     */
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Matrix initialBias, final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
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
    public KnownPositionAccelerometerCalibrator(
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
    public KnownPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public KnownPositionAccelerometerCalibrator(final ECEFPosition position) {
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(position);
        try {
            setListener(listener);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements) {
        this(position);
        try {
            setMeasurements(measurements);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final boolean commonAxisUsed) {
        this(commonAxisUsed);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final boolean commonAxisUsed,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY,
                initialBiasZ);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy,
                initialSz);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final double[] initialBias) {
        this(initialBias);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(initialBias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        this(measurements, initialBias);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final boolean commonAxisUsed,
            final double[] initialBias) {
        this(commonAxisUsed, initialBias);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final boolean commonAxisUsed,
            final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        this(measurements, commonAxisUsed, initialBias);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Matrix initialBias) {
        this(initialBias);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial bias to find a solution.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(initialBias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        this(measurements, initialBias);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(commonAxisUsed, initialBias);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(measurements, commonAxisUsed, initialBias);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Matrix initialBias, final Matrix initialMa) {
        this(initialBias, initialMa);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Matrix initialBias, final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(initialBias, initialMa, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        this(measurements, initialBias, initialMa);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, initialBias, initialMa, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        this(commonAxisUsed, initialBias, initialMa);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, initialBias, initialMa, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        this(measurements, commonAxisUsed, initialBias, initialMa);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias, initialMa, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public KnownPositionAccelerometerCalibrator(final NEDPosition position) {
        this(convertPosition(position));
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements) {
        this(convertPosition(position), measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position, final boolean commonAxisUsed) {
        this(convertPosition(position), commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position, final boolean commonAxisUsed,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(convertPosition(position), measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(convertPosition(position), initialBiasX, initialBiasY,
                initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), initialBiasX, initialBiasY,
                initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(convertPosition(position), measurements, initialBiasX,
                initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialBiasX,
                initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(convertPosition(position), commonAxisUsed, initialBiasX,
                initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed, initialBiasX,
                initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(convertPosition(position), initialBiasX, initialBiasY,
                initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), initialBiasX, initialBiasY,
                initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        this(convertPosition(position), measurements, initialBiasX,
                initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialBiasX,
                initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(convertPosition(position), commonAxisUsed, initialBiasX,
                initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed, initialBiasX,
                initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertPosition(position), initialBiasX, initialBiasY,
                initialBiasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertPosition(position), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertPosition(position), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertPosition(position), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertPosition(position), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(convertPosition(position), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertPosition(position), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertPosition(position), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertPosition(position), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(convertPosition(position), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(convertPosition(position), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(convertPosition(position), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position, final double[] initialBias) {
        this(convertPosition(position), initialBias);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position, final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        this(convertPosition(position), measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position, final boolean commonAxisUsed,
            final double[] initialBias) {
        this(convertPosition(position), commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position, final boolean commonAxisUsed,
            final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed, initialBias,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, initialBias,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Matrix initialBias) {
        this(convertPosition(position), initialBias);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial bias to find a solution.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        this(convertPosition(position), measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialBias,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(convertPosition(position), commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Matrix initialBias, final Matrix initialMa) {
        this(convertPosition(position), initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures have been taken.
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Matrix initialBias, final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        this(convertPosition(position), measurements, initialBias,
                initialMa);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialBias,
                initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        this(convertPosition(position), commonAxisUsed, initialBias,
                initialMa);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), commonAxisUsed, initialBias,
                initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
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
    public KnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final KnownPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialBias, initialMa, listener);
    }

    /**
     * Gets position where body kinematics measures have been taken expressed in
     * ECEF coordinates.
     *
     * @return position where body kinematics measures have been taken.
     */
    public ECEFPosition getEcefPosition() {
        return mPosition;
    }

    /**
     * Sets position where body kinematics measures have been taken expressed in
     * ECEF coordinates.
     *
     * @param position position where body kinematics measures have been taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final ECEFPosition position) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }

        mPosition = position;
        if (position != null) {
            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                    mPosition.getX(), mPosition.getY(), mPosition.getZ());
            mGroundTruthGravityNorm = gravity.getNorm();
        } else {
            mGroundTruthGravityNorm = null;
        }
    }

    /**
     * Gets position where body kinematics measures have been taken expressed in
     * NED coordinates.
     *
     * @return position where body kinematics measures have been taken or null if
     * not available.
     */
    public NEDPosition getNedPosition() {
        final NEDPosition result = new NEDPosition();
        return getNedPosition(result) ? result : null;
    }

    /**
     * Gets position where body kinematics measures have been taken expressed in
     * NED coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if NED position could be computed, false otherwise.
     */
    public boolean getNedPosition(final NEDPosition result) {

        if (mPosition != null) {
            final NEDVelocity velocity = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                    mPosition.getX(), mPosition.getY(), mPosition.getZ(),
                    0.0, 0.0, 0.0, result, velocity);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets position where body kinematics measures have been taken expressed in
     * NED coordinates.
     *
     * @param position position where body kinematics measures have been taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final NEDPosition position) throws LockedException {
        setPosition(position != null ? convertPosition(position) : null);
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mPosition != null;
    }

    /**
     * Converts provided NED position expressed in terms of latitude, longitude and height respect
     * mean Earth surface, to position expressed in ECEF coordinates.
     *
     * @param position NED position to be converted.
     * @return converted position expressed in ECEF coordinates.
     */
    private static ECEFPosition convertPosition(final NEDPosition position) {
        final ECEFVelocity velocity = new ECEFVelocity();
        final ECEFPosition result = new ECEFPosition();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                position.getLatitude(), position.getLongitude(), position.getHeight(),
                0.0, 0.0, 0.0, result, velocity);
        return result;
    }
}
