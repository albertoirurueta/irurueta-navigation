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
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates accelerometer biases, cross couplings and scaling factors
 * using a MSAC algorithm to discard outliers.
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
public class MSACRobustKnownPositionAccelerometerCalibrator extends
        RobustKnownPositionAccelerometerCalibrator {

    /**
     * Constant defining default threshold to determine whether samples are
     * inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 1e-2;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether samples are inliers or not when
     * testing possible estimation solutions.
     */
    private double mThreshold = DEFAULT_THRESHOLD;

    /**
     * Constructor.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final List<StandardDeviationBodyKinematics> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final double[] initialBias) {
        super(initialBias);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(final Matrix initialBias) {
        super(initialBias);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final Matrix initialBias, final Matrix initialMa) {
        super(initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position) {
        super(position);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements) {
        super(position, measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        super(position, measurements, initialBias);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        super(position, measurements, initialBias);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, initialBias, initialMa);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, initialMa, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position) {
        super(position);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements) {
        super(position, measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     */
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        super(position, measurements, initialBias);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        super(position, measurements, initialBias);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, initialBias, initialMa);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, initialMa, listener);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa);
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
    public MSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa,
                listener);
    }

    /**
     * Returns threshold to determine whether samples are inliers or not.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not.
     *
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     *                                  zero.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setThreshold(double threshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }

    /**
     * Estimates accelerometer calibration parameters containing bias, scale factors
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

        mGravityNorm = computeGravityNorm();

        final MSACRobustEstimator<PreliminaryResult> innerEstimator =
                new MSACRobustEstimator<>(new MSACRobustEstimatorListener<PreliminaryResult>() {
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
                    public void estimatePreliminarSolutions(final int[] samplesIndices,
                                                            final List<PreliminaryResult> solutions) {
                        computePreliminarySolutions(samplesIndices, solutions);
                    }

                    @Override
                    public double computeResidual(final PreliminaryResult currentEstimation, final int i) {
                        return computeError(mMeasurements.get(i), currentEstimation);
                    }

                    @Override
                    public boolean isReady() {
                        return MSACRobustKnownPositionAccelerometerCalibrator.super.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<PreliminaryResult> estimator) {
                        if (mListener != null) {
                            mListener.onCalibrateStart(MSACRobustKnownPositionAccelerometerCalibrator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<PreliminaryResult> estimator) {
                        if (mListener != null) {
                            mListener.onCalibrateEnd(MSACRobustKnownPositionAccelerometerCalibrator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<PreliminaryResult> estimator, final int iteration) {
                        if (mListener != null) {
                            mListener.onCalibrateNextIteration(
                                    MSACRobustKnownPositionAccelerometerCalibrator.this, iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<PreliminaryResult> estimator, final float progress) {
                        if (mListener != null) {
                            mListener.onCalibrateProgressChange(
                                    MSACRobustKnownPositionAccelerometerCalibrator.this, progress);
                        }
                    }
                });

        try {
            mRunning = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            final PreliminaryResult preliminaryResult = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();

            attemptRefine(preliminaryResult);

        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } catch (RobustEstimatorException e) {
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
        return RobustEstimatorMethod.MSAC;
    }
}
