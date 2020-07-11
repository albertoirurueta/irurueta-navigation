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
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates gyroscope cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer using PROMedS robust estimator.
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
public class PROMedSRobustKnownBiasEasyGyroscopeCalibrator extends
        RobustKnownBiasEasyGyroscopeCalibrator {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used to
     * avoid keeping the algorithm unnecessarily iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once a
     * solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-3;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double mStopThreshold = DEFAULT_STOP_THRESHOLD;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, bias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @param listener    listener to handle events raised by this
     *                    calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, bias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @param listener    listener to handle events raised by this
     *                    calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
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
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores) {
        super();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param sequences     collection of sequences containing timestamped body
     *                      kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg     initial gyroscope scale factors and cross coupling
     *                      errors matrix. Must be 3x3.
     * @param initialGg     initial gyroscope G-dependent cross biases
     *                      introduced on the gyroscope by the specific forces
     *                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, bias, initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param sequences     collection of sequences containing timestamped body
     *                      kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg     initial gyroscope scale factors and cross coupling
     *                      errors matrix. Must be 3x3.
     * @param initialGg     initial gyroscope G-dependent cross biases
     *                      introduced on the gyroscope by the specific forces
     *                      sensed by the accelerometer. Must be 3x3.
     * @param listener      listener to handle events raised by this
     *                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, bias, initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param sequences     collection of sequences containing timestamped body
     *                      kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg     initial gyroscope scale factors and cross coupling
     *                      errors matrix. Must be 3x3.
     * @param initialGg     initial gyroscope G-dependent cross biases
     *                      introduced on the gyroscope by the specific forces
     *                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, bias, initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param sequences     collection of sequences containing timestamped body
     *                      kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg     initial gyroscope scale factors and cross coupling
     *                      errors matrix. Must be 3x3.
     * @param initialGg     initial gyroscope G-dependent cross biases
     *                      introduced on the gyroscope by the specific forces
     *                      sensed by the accelerometer. Must be 3x3.
     * @param listener      listener to handle events raised by this
     *                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, bias, initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sequence. The larger the score value the better
     *                          the quality of the sequence.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        super(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sequence. The larger the score value the better
     *                          the quality of the sequence.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sequence. The larger the score value the better
     *                          the quality of the sequence.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        super(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sequence. The larger the score value the better
     *                          the quality of the sequence.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        super(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
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
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
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
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
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
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
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
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
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
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
            final double[] qualityScores,
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
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algrithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return mStopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value,
     * the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setStopThreshold(double stopThreshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = stopThreshold;
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
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mSequences.size();
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
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (mRunning) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final PROMedSRobustEstimator<PreliminaryResult> innerEstimator =
                new PROMedSRobustEstimator<>(new PROMedSRobustEstimatorListener<PreliminaryResult>() {
                    @Override
                    public double[] getQualityScores() {
                        return mQualityScores;
                    }

                    @Override
                    public double getThreshold() {
                        return mStopThreshold;
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
                        return PROMedSRobustKnownBiasEasyGyroscopeCalibrator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(
                            final RobustEstimator<PreliminaryResult> estimator) {
                        if (mListener != null) {
                            mListener.onCalibrateStart(
                                    PROMedSRobustKnownBiasEasyGyroscopeCalibrator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(
                            final RobustEstimator<PreliminaryResult> estimator) {
                        if (mListener != null) {
                            mListener.onCalibrateEnd(
                                    PROMedSRobustKnownBiasEasyGyroscopeCalibrator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<PreliminaryResult> estimator,
                            final int iteration) {
                        if (mListener != null) {
                            mListener.onCalibrateNextIteration(
                                    PROMedSRobustKnownBiasEasyGyroscopeCalibrator.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<PreliminaryResult> estimator,
                            final float progress) {
                        if (mListener != null) {
                            mListener.onCalibrateProgressChange(
                                    PROMedSRobustKnownBiasEasyGyroscopeCalibrator.this,
                                    progress);
                        }
                    }
                });

        try {
            mRunning = true;

            setupAccelerationFixer();

            mInliersData = null;
            innerEstimator.setUseInlierThresholds(true);
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
        } catch (RobustEstimatorException | AlgebraException e) {
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
        return RobustEstimatorMethod.PROMedS;
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
