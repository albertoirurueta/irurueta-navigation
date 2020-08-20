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
 * Estimates accelerometer cross couplings and scaling factors when accelerometer biases
 * are known.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 7 measurements taken at a single known position must
 * be taken at 7 different unknown orientations and zero velocity when common z-axis
 * is assumed, otherwise at least 10 measurements are required.
 * <p>
 * Measured specific force is assumed to follow the model shown below:
 * <pre>
 *     fmeas = ba + (I + Ma) * ftrue + w
 * </pre>
 * Where:
 * - fmeas is the measured specific force. This is a 3x1 vector.
 * - ba is accelerometer bias, which is known. This is a 3x1 vector.
 * - I is the 3x3 identity matrix.
 * - Ma is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect accelerometer, this should be a 3x3 zero matrix.
 * - ftrue is ground-trush specific force.
 * - w is measurement noise.
 */
public class KnownBiasAndPositionAccelerometerCalibrator extends
        BaseBiasGravityNormAccelerometerCalibrator<KnownBiasAndPositionAccelerometerCalibrator,
                KnownBiasAndPositionAccelerometerCalibrationListener> {

    /**
     * Position where body kinematics measures have been taken.
     */
    private ECEFPosition mPosition;

    /**
     * Constructor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
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
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX x-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     * @param biasY y-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     * @param biasZ z-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ) {
        super(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    x-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasY    y-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasZ    z-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ) {
        super(measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        super(commonAxisUsed, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX x-coordinate of accelerometer bias.
     * @param biasY y-coordinate of accelerometer bias.
     * @param biasZ z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        super(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    x-coordinate of accelerometer bias.
     * @param biasY    y-coordinate of accelerometer bias.
     * @param biasZ    z-coordinate of accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        super(measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        super(commonAxisUsed, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasY     y-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasZ     z-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(biasX, biasY, biasZ, initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        super(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX      x-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasY      y-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        super(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @param listener   listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        super(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        super(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias. This must have length 3 and is expressed
     *             in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final double[] bias) {
        super(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias. This must have length 3 and is expressed
     *                 in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(bias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        super(measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] bias) {
        super(commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        super(measurements, commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(final Matrix bias) {
        super(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(bias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        super(measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias) {
        super(commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        super(measurements, commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Matrix bias, final Matrix initialMa) {
        super(bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        super(measurements, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        super(commonAxisUsed, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(commonAxisUsed, bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        super(measurements, commonAxisUsed, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        super(measurements, commonAxisUsed, bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(final ECEFPosition position) {
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
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
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
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
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position, final boolean commonAxisUsed,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
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
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
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
     * @param position position where body kinematics measures have been taken.
     * @param biasX    x-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasY    y-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasZ    z-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final double biasX, final double biasY,
            final double biasZ) {
        this(biasX, biasY, biasZ);
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
     * @param biasX    x-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasY    y-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasZ    z-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(biasX, biasY, biasZ, listener);
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ) {
        this(measurements, biasX, biasY, biasZ);
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(measurements, commonAxisUsed, biasX, biasY,
                biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed,
                biasX, biasY, biasZ, listener);
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
     * @param biasX    x-coordinate of accelerometer bias.
     * @param biasY    y-coordinate of accelerometer bias.
     * @param biasZ    z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
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
     * @param biasX    x-coordinate of accelerometer bias.
     * @param biasY    y-coordinate of accelerometer bias.
     * @param biasZ    z-coordinate of accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(biasX, biasY, biasZ, listener);
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
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        this(measurements, biasX, biasY, biasZ);
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
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(measurements, commonAxisUsed,
                biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param position  position where body kinematics measures have been taken.
     * @param biasX     x-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasY     y-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasZ     z-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy,
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param position  position where body kinematics measures have been taken.
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ,
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
     * @param position  position where body kinematics measures have been taken.
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param position   position where body kinematics measures have been taken.
     * @param biasX      x-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasY      y-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param position   position where body kinematics measures have been taken.
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ,
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
     * @param position   position where body kinematics measures have been taken.
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @param listener   listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param position position where body kinematics measures have been taken.
     * @param bias     known accelerometer bias. This must have length 3 and is expressed
     *                 in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position, final double[] bias) {
        this(bias);
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
     * @param bias     known accelerometer bias. This must have length 3 and is expressed
     *                 in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position, final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(bias, listener);
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
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        this(measurements, bias);
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
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, bias, listener);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position, final boolean commonAxisUsed,
            final double[] bias) {
        this(commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position, final boolean commonAxisUsed,
            final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, bias, listener);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(measurements, commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed, bias, listener);
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
     * @param bias     known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Matrix bias) {
        this(bias);
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
     * @param bias     known accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(bias, listener);
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
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        this(measurements, bias);
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
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, bias, listener);
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
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Matrix bias) {
        this(commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, bias, listener);
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
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(measurements, commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed, bias, listener);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position  position where body kinematics measures have been taken.
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Matrix bias, final Matrix initialMa) {
        this(bias, initialMa);
        try {
            setPosition(position);
        } catch (LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position  position where body kinematics measures have been taken.
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(bias, initialMa, listener);
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
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(measurements, bias, initialMa);
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
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, bias, initialMa, listener);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(commonAxisUsed, bias, initialMa);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(commonAxisUsed, bias, initialMa, listener);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(measurements, commonAxisUsed, bias, initialMa);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(measurements, commonAxisUsed, bias, initialMa, listener);
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
    public KnownBiasAndPositionAccelerometerCalibrator(final NEDPosition position) {
        this(convertPosition(position));
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
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
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position, final boolean commonAxisUsed,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
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
    public KnownBiasAndPositionAccelerometerCalibrator(
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param biasX    x-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasY    y-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasZ    z-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final double biasX, final double biasY,
            final double biasZ) {
        this(convertPosition(position), biasX, biasY,
                biasZ);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param biasX    x-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasY    y-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasZ    z-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), biasX, biasY,
                biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ) {
        this(convertPosition(position), measurements, biasX,
                biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, biasX,
                biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(convertPosition(position), commonAxisUsed, biasX,
                biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed, biasX,
                biasY, biasZ, listener);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param biasX    x-coordinate of accelerometer bias.
     * @param biasY    y-coordinate of accelerometer bias.
     * @param biasZ    z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(convertPosition(position), biasX, biasY,
                biasZ);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param biasX    x-coordinate of accelerometer bias.
     * @param biasY    y-coordinate of accelerometer bias.
     * @param biasZ    z-coordinate of accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), biasX, biasY,
                biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        this(convertPosition(position), measurements, biasX,
                biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, biasX,
                biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(convertPosition(position), commonAxisUsed, biasX,
                biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed, biasX,
                biasY, biasZ, listener);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param position  position where body kinematics measures have been taken.
     * @param biasX     x-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasY     y-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasZ     z-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertPosition(position), biasX, biasY,
                biasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertPosition(position), measurements,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertPosition(position), commonAxisUsed,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position  position where body kinematics measures have been taken.
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertPosition(position), biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position  position where body kinematics measures have been taken.
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertPosition(position), measurements,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(convertPosition(position), commonAxisUsed,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param position   position where body kinematics measures have been taken.
     * @param biasX      x-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasY      y-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertPosition(position), biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertPosition(position), measurements,
                biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertPosition(position), commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position   position where body kinematics measures have been taken.
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(convertPosition(position), biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param position   position where body kinematics measures have been taken.
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @param listener   listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(convertPosition(position), measurements,
                biasX, biasY, biasZ,
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
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(convertPosition(position), commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ,
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param bias     known accelerometer bias. This must have length 3 and is expressed
     *                 in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position, final double[] bias) {
        this(convertPosition(position), bias);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param bias     known accelerometer bias. This must have length 3 and is expressed
     *                 in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position, final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), bias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        this(convertPosition(position), measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position, final boolean commonAxisUsed,
            final double[] bias) {
        this(convertPosition(position), commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position, final boolean commonAxisUsed,
            final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed, bias,
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(convertPosition(position), measurements, commonAxisUsed,
                bias);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, bias,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param bias     known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Matrix bias) {
        this(convertPosition(position), bias);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     * @param bias     known accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), bias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        this(convertPosition(position), measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, bias,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Matrix bias) {
        this(convertPosition(position), commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed, bias, listener);
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
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(convertPosition(position), measurements, commonAxisUsed,
                bias);
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
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                bias, listener);
    }

    /**
     * Constructor.
     *
     * @param position  position where body kinematics measures have been taken.
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Matrix bias, final Matrix initialMa) {
        this(convertPosition(position), bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position  position where body kinematics measures have been taken.
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(convertPosition(position), measurements, bias,
                initialMa);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, bias,
                initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(convertPosition(position), commonAxisUsed, bias,
                initialMa);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), commonAxisUsed, bias,
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(convertPosition(position), measurements, commonAxisUsed,
                bias, initialMa);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndPositionAccelerometerCalibrationListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                bias, initialMa, listener);
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
     * Gets position where body kinematics measures have been taken expressed in
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
