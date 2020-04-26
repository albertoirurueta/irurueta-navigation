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
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates gyroscope biases, cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer using PROSAC robust estimator.
 * <p>
 * This calibrator assumes that the IMU is placed flat on a turntable spinning
 * at constant speed, but absolute orientation or position of IMU is unknown.
 * Turntable must rotate fast enough so that Earth rotation effects can be
 * neglected, bus slow enough so that gyroscope readings can be properly made.
 * <p>
 * To use this calibrator at least 10 measurements are needed when common
 * z-axis is assumed and G-dependent cross biases are ignored, otherwise
 * at least 13 measurements are required when common z-axis is not assumed.
 * If G-dependent cross biases are being estimated, then at least 19
 * measurements are needed when common z-axis is assumed, otherwise at
 * least 22 measurements are required when common z-axis is not assumed.
 * <p>
 * Measured gyroscope angular rates is assumed to follow the model shown below:
 * <pre>
 *     Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w
 * </pre>
 * Where:
 * - Ωmeas is the measured gyroscope angular rates. This is a 3x1 vector.
 * - bg is the gyroscope bias. Ideally, on a perfect gyroscope, this should be a
 * 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Mg is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect gyroscope, this should be a 3x3 zero matrix.
 * - Ωtrue is ground-truth gyroscope angular rates.
 * - Gg is the G-dependent cross biases introduced by the specific forces sensed
 * by the accelerometer. Ideally, on a perfect gyroscope, this should be a 3x3
 * zero matrix.
 * - ftrue is ground-truth specific force. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 */
public class PROSACRobustTurntableGyroscopeCalibrator extends
        RobustTurntableGyroscopeCalibrator {

    /**
     * Constant defining default threshold to determine whether samples are inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 5e-1;

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
    public PROSACRobustTurntableGyroscopeCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval,
                measurements, initialBias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by this
     *                              calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @param listener                      listener to handle events
     *                                      raised by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated,
     *                                      false otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must
     *                                      have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must be
     *                                      3x1 and is expressed in radians
     *                                      per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by this
     *                              calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @param listener                      listener to handle events
     *                                      raised by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated,
     *                                      false otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must
     *                                      have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must be
     *                                      3x1 and is expressed in radians
     *                                      per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores) {
        super();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval,
                measurements, initialBias, initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by this
     *                              calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @param listener                      listener to handle events
     *                                      raised by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated,
     *                                      false otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must
     *                                      have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.*
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must be
     *                                      3x1 and is expressed in radians
     *                                      per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.*
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.*
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by this
     *                              calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.*
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.*
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.*
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.*
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.*
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores         quality scores corresponding to each provided
     *                              measurement. The larger the score value the better
     *                              the quality of the sample.
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @param listener                      listener to handle events
     *                                      raised by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated,
     *                                      false otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must
     *                                      have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      measurement. The larger the score value the better
     *                                      the quality of the sample.
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must be
     *                                      3x1 and is expressed in radians
     *                                      per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROSACRobustTurntableGyroscopeCalibrator(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustTurntableGyroscopeCalibratorListener listener) {
        super(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener);
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
    public void setQualityScores(double[] qualityScores)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether calibrator is ready to find a solution.
     *
     * @return true if calibrator is ready, false otherwise.
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
    public void setComputeAndKeepInliersEnabled(boolean computeAndKeepInliers)
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
    public void setComputeAndKeepResidualsEnabled(boolean computeAndKeepResiduals)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mComputeAndKeepResiduals = computeAndKeepResiduals;
    }

    /**
     * Estimates gyroscope calibration parameters containing bias, scale factors
     * cross-coupling errors and g-dependant cross biases.
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

        final PROSACRobustEstimator<PreliminaryResult> innerEstimator =
                new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<PreliminaryResult>() {
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
                            final int[] samplesIndices, final List<PreliminaryResult> solutions) {
                        computePreliminarySolutions(samplesIndices, solutions);
                    }

                    @Override
                    public double computeResidual(
                            final PreliminaryResult currentEstimation, final int i) {
                        return computeError(mMeasurements.get(i), currentEstimation);
                    }

                    @Override
                    public boolean isReady() {
                        return PROSACRobustTurntableGyroscopeCalibrator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<PreliminaryResult> estimator) {
                        if (mListener != null) {
                            mListener.onCalibrateStart(PROSACRobustTurntableGyroscopeCalibrator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<PreliminaryResult> estimator) {
                        if (mListener != null) {
                            mListener.onCalibrateEnd(PROSACRobustTurntableGyroscopeCalibrator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<PreliminaryResult> estimator, final int iteration) {
                        if (mListener != null) {
                            mListener.onCalibrateNextIteration(
                                    PROSACRobustTurntableGyroscopeCalibrator.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<PreliminaryResult> estimator, final float progress) {
                        if (mListener != null) {
                            mListener.onCalibrateProgressChange(
                                    PROSACRobustTurntableGyroscopeCalibrator.this, progress);
                        }
                    }
                });

        try {
            mRunning = true;
            mInliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(
                    mComputeAndKeepInliers || mRefineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(
                    mComputeAndKeepResiduals || mRefineResult);
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
                qualityScores.length < TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }
}
