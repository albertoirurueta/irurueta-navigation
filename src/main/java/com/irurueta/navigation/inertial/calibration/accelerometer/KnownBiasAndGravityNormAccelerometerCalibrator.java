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
 * Estimates accelerometer cross couplings and scaling factors when accelerometer biases
 * are known.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 7 measurements taken at a single position where
 * gravity norm is known must be taken at 7 different unknown orientations and
 * zero velocity when common z-axis is assumed, otherwise at least 10 measurements
 * are required.
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
public class KnownBiasAndGravityNormAccelerometerCalibrator extends
        BaseBiasGravityNormAccelerometerCalibrator<KnownBiasAndGravityNormAccelerometerCalibrator,
                KnownBiasAndGravityNormAccelerometerCalibratorListener> {

    /**
     * Constructor.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX x-coordinate of accelerometer bias.
     * @param biasY y-coordinate of accelerometer bias.
     * @param biasZ z-coordinate of accelerometer bias.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(final Matrix bias) {
        super(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double[] bias) {
        super(groundTruthGravityNorm, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        super(groundTruthGravityNorm, measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double[] bias) {
        super(groundTruthGravityNorm, commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, bias, listener);
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
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, bias);
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
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, bias,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix bias) {
        super(groundTruthGravityNorm, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        super(groundTruthGravityNorm, measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias) {
        super(groundTruthGravityNorm, commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, bias, listener);
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
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, bias);
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
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, bias,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix bias, final Matrix initialMa) {
        super(groundTruthGravityNorm, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, bias, initialMa,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        super(groundTruthGravityNorm, commonAxisUsed, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, commonAxisUsed, bias,
                initialMa, listener);
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
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, bias,
                initialMa);
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
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, bias,
                initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm) {
        this(convertAcceleration(groundTruthGravityNorm));
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm,
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed) {
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                listener);
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed);
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                biasX, biasY, biasZ, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration biasX,
            final Acceleration biasY,
            final Acceleration biasZ) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration biasX,
            final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                biasX, biasY, biasZ, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        super(convertAcceleration(groundTruthGravityNorm),
                biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, biasX, biasY, biasZ,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed,
                biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final double[] bias) {
        this(convertAcceleration(groundTruthGravityNorm), bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements,
                bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double[] bias) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, bias, listener);
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
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed, bias);
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
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Matrix bias) {
        this(convertAcceleration(groundTruthGravityNorm), bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, bias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, bias, listener);
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
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed, bias);
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
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Matrix bias, final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm), bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, bias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                commonAxisUsed, bias, initialMa, listener);
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
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed, bias, initialMa);
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
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public KnownBiasAndGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final KnownBiasAndGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm),
                measurements, commonAxisUsed, bias, initialMa, listener);
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
