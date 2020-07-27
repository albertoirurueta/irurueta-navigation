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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates gyroscope cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer using MSAC robust estimator.
 * <p>
 * This calibrator assumes that the IMU is at a more or less fixed location on
 * Earth, and evaluates sequences of measured body kinematics to perform
 * calibration for unknown orientations on those provided sequences.
 * Each provided sequence will be preceded by a static period where mean
 * specific force will be measured to determine gravity (and hence partial
 * body attitude).
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
public class MSACRobustKnownBiasEasyGyroscopeCalibrator extends
        RobustKnownBiasEasyGyroscopeCalibrator {

    /**
     * Constant defining default threshold to determine whether samples are
     * inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 1e-3;

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
    public MSACRobustKnownBiasEasyGyroscopeCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param sequences collection of sequences containing timestamped body
     *                  kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg initial gyroscope scale factors and cross coupling
     *                  errors matrix. Must be 3x3.
     * @param initialGg initial gyroscope G-dependent cross biases
     *                  introduced on the gyroscope by the specific forces
     *                  sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, bias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param sequences collection of sequences containing timestamped body
     *                  kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg initial gyroscope scale factors and cross coupling
     *                  errors matrix. Must be 3x3.
     * @param initialGg initial gyroscope G-dependent cross biases
     *                  introduced on the gyroscope by the specific forces
     *                  sensed by the accelerometer. Must be 3x3.
     * @param listener  listener to handle events raised by this
     *                  calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, bias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param sequences collection of sequences containing timestamped body
     *                  kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg initial gyroscope scale factors and cross coupling
     *                  errors matrix. Must be 3x3.
     * @param initialGg initial gyroscope G-dependent cross biases
     *                  introduced on the gyroscope by the specific forces
     *                  sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, bias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param sequences collection of sequences containing timestamped body
     *                  kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg initial gyroscope scale factors and cross coupling
     *                  errors matrix. Must be 3x3.
     * @param initialGg initial gyroscope G-dependent cross biases
     *                  introduced on the gyroscope by the specific forces
     *                  sensed by the accelerometer. Must be 3x3.
     * @param listener  listener to handle events raised by this
     *                  calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, bias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias              gyroscope known bias. This must have length 3 and is
     *                          expressed in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias              gyroscope known bias. This must have length 3 and is
     *                          expressed in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias              gyroscope known bias. This must be 3x1 and is
     *                          expressed in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias              gyroscope known bias. This must be 3x1 and is
     *                          expressed in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must be 3x1 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must be 3x1 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must have length 3 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must have length 3 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must have length 3 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must have length 3 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must be 3x1 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must be 3x1 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public MSACRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
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
     * Estimates gyroscope calibration parameters containing scale factors,
     * cross-coupling errors and G-dependent coupling.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    @Override
    public void calibrate() throws LockedException, NotReadyException,
            CalibrationException {
        if (mRunning) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final MSACRobustEstimator<PreliminaryResult> innerEstimator =
                new MSACRobustEstimator<>(
                        new MSACRobustEstimatorListener<PreliminaryResult>() {
                    @Override
                    public double getThreshold() {
                        return mThreshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return mSequences.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return mPreliminarySubsetSize;
                    }

                    @Override
                    public void estimatePreliminarSolutions(
                            final int[] samplesIndices,
                            final List<PreliminaryResult> solutions) {
                        computePreliminarySolutions(samplesIndices, solutions);
                    }

                    @Override
                    public double computeResidual(
                            final PreliminaryResult currentEstimation,
                            final int i) {
                        return computeError(mSequences.get(i), currentEstimation);
                    }

                    @Override
                    public boolean isReady() {
                        return MSACRobustKnownBiasEasyGyroscopeCalibrator.super.isReady();
                    }

                    @Override
                    public void onEstimateStart(
                            final RobustEstimator<PreliminaryResult> estimator) {
                    }

                    @Override
                    public void onEstimateEnd(
                            final RobustEstimator<PreliminaryResult> estimator) {
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<PreliminaryResult> estimator,
                            final int iteration) {
                        if (mListener != null) {
                            mListener.onCalibrateNextIteration(
                                    MSACRobustKnownBiasEasyGyroscopeCalibrator.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<PreliminaryResult> estimator,
                            final float progress) {
                        if (mListener != null) {
                            mListener.onCalibrateProgressChange(
                                    MSACRobustKnownBiasEasyGyroscopeCalibrator.this,
                                    progress);
                        }
                    }
                });

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onCalibrateStart(this);
            }

            setupAccelerationFixer();

            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            final PreliminaryResult preliminaryResult = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();

            attemptRefine(preliminaryResult);

            if (mListener != null) {
                mListener.onCalibrateEnd(this);
            }

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } catch (final RobustEstimatorException | AlgebraException e) {
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
