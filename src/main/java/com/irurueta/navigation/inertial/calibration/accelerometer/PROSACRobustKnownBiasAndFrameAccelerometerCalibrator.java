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
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;

import java.util.List;

/**
 * Robustly estimates accelerometer cross couplings and scaling factors using
 * a PROSAC algorithm to discard outliers.
 * This estimator assumes that biases are known.
 * <p>
 * To use this calibrator at least 4 measurements at different known frames must
 * be provided. In other words, accelerometer samples must be obtained at 4
 * different positions, orientations and velocities (although typically velocities are
 * always zero).
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
public class PROSACRobustKnownBiasAndFrameAccelerometerCalibrator extends
        RobustKnownBiasAndFrameAccelerometerCalibrator {

    /**
     * Constant defining default threshold to determine whether samples are inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 1e-2;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;

    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;

    /**
     * Threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and
     * distances provided for each sample.
     */
    private double mThreshold = DEFAULT_THRESHOLD;

    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
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
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasY known y coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasZ known z coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        super(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasY    known y coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasZ    known z coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        super(measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        super(biasX, biasY, biasZ, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(biasX, biasY, biasZ, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @param biasY known y coordinate of accelerometer bias.
     * @param biasZ known z coordinate of accelerometer bias.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        super(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of accelerometer bias.
     * @param biasY    known y coordinate of accelerometer bias.
     * @param biasZ    known z coordinate of accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        super(measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        super(biasX, biasY, biasZ, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(biasX, biasY, biasZ, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] bias) {
        super(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(bias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias) {
        super(measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] bias, final boolean commonAxisUsed) {
        super(bias, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(bias, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final boolean commonAxisUsed) {
        super(measurements, bias, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, bias, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final Matrix bias) {
        super(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final Matrix bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(bias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias) {
        super(measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final Matrix bias, final boolean commonAxisUsed) {
        super(bias, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(bias, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final boolean commonAxisUsed) {
        super(measurements, bias, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        super(measurements, bias, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        this(measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasY         known y coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasZ         known z coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasY         known y coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasZ         known z coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(biasX, biasY, biasZ, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasY         known y coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasZ         known z coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(measurements, biasX, biasY, biasZ);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasY         known y coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasZ         known z coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        this(biasX, biasY, biasZ, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of accelerometer bias.
     * @param biasY         known y coordinate of accelerometer bias.
     * @param biasZ         known z coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of accelerometer bias.
     * @param biasY         known y coordinate of accelerometer bias.
     * @param biasZ         known z coordinate of accelerometer bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(biasX, biasY, biasZ, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of accelerometer bias.
     * @param biasY         known y coordinate of accelerometer bias.
     * @param biasZ         known z coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(measurements, biasX, biasY, biasZ);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of accelerometer bias.
     * @param biasY         known y coordinate of accelerometer bias.
     * @param biasZ         known z coordinate of accelerometer bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        this(biasX, biasY, biasZ, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known accelerometer bias.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores, final double[] bias) {
        this(bias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known accelerometer bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final double[] bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(bias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known accelerometer bias.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias) {
        this(measurements, bias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known accelerometer bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, bias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores, final double[] bias,
            final boolean commonAxisUsed) {
        this(bias, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores, final double[] bias,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(bias, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final boolean commonAxisUsed) {
        this(measurements, bias, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, bias, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores, final Matrix bias) {
        this(bias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known accelerometer bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final Matrix bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(bias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias) {
        this(measurements, bias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known accelerometer bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, bias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores, final Matrix bias,
            final boolean commonAxisUsed) {
        this(bias, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores, final Matrix bias,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(bias, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final boolean commonAxisUsed) {
        this(measurements, bias, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, bias, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on norm between measured specific forces and the
     * ones generated with estimated calibration parameters provided for each sample.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on norm between measured specific forces and the
     * ones generated with estimated calibration parameters provided for each sample.
     *
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }

    /**
     * Returns quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     *
     * @return quality scores corresponding to each sample.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     *
     * @param qualityScores quality scores corresponding to each sample.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setQualityScores(final double[] qualityScores)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mMeasurements.size();
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return mComputeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if calibrator is currently running.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mComputeAndKeepInliers = computeAndKeepInliers;
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResiduals() {
        return mComputeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if calibrator is currently running.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mComputeAndKeepResiduals = computeAndKeepResiduals;
    }

    /**
     * Estimates accelerometer calibration parameters containing scale factors
     * and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    @Override
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (mRunning) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final PROSACRobustEstimator<Matrix> innerEstimator =
                new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<Matrix>() {
                    @Override
                    public double[] getQualityScores() {
                        return mQualityScores;
                    }

                    @Override
                    public double getThreshold() {
                        return mThreshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return mMeasurements.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return mPreliminarySubsetSize;
                    }

                    @Override
                    public void estimatePreliminarSolutions(
                            final int[] samplesIndices,
                            final List<Matrix> solutions) {
                        computePreliminarySolutions(samplesIndices, solutions);
                    }

                    @Override
                    public double computeResidual(
                            final Matrix currentEstimation,
                            final int i) {
                        return computeError(mMeasurements.get(i), currentEstimation);
                    }

                    @Override
                    public boolean isReady() {
                        return PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(
                            final RobustEstimator<Matrix> estimator) {
                    }

                    @Override
                    public void onEstimateEnd(
                            final RobustEstimator<Matrix> estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<Matrix> estimator,
                            final int iteration) {
                        if (mListener != null) {
                            mListener.onCalibrateNextIteration(
                                    PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<Matrix> estimator,
                            final float progress) {
                        if (mListener != null) {
                            mListener.onCalibrateProgressChange(
                                    PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.this,
                                    progress);
                        }
                    }
                });

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onCalibrateStart(this);
            }

            mInliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(
                    mComputeAndKeepInliers || mRefineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(
                    mComputeAndKeepResiduals || mRefineResult);
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            final Matrix preliminaryResult = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();

            attemptRefine(preliminaryResult);

            if (mListener != null) {
                mListener.onCalibrateEnd(this);
            }

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } catch (final RobustEstimatorException e) {
            throw new CalibrationException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null ||
                qualityScores.length < MINIMUM_MEASUREMENTS) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }
}
