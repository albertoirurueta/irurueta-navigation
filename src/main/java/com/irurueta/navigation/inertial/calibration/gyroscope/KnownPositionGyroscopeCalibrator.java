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
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.AccelerationFixer;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.Collection;
import java.util.List;

/**
 * Estimates gyroscope biases, cross couplings, scaling factors and
 * G-dependent cross biases introduced on the gyroscope by the specific
 * forces sensed by the accelerometer.
 * <p>
 * This calibrator assumes that the IMU is at a more or less fixed location on
 * Earth (to account for gravitational effects), and evaluates sequences of
 * measured body kinematics to perform calibration for unknown orientations
 * on those provided sequences.
 * <p>
 * To use this calibrator at least 10 sequences are needed (each containing
 * at least 3 body kinematics measurements) when common z-axis is assumed and
 * G-dependant cross biases are ignored, otherwise at least 13 sequences are
 * required (each containing at least 3 body kinematics measurements) when
 * common z-axis is not assumed.
 * If G-dependent cross biases are being estimated, then at least 19
 * measurements are needed when common z-axis is assumed, otherwise at least
 * 22 sequences are required (each containing at least 3 body kinematics
 * measurements) when common z-axis is not assumed.
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
public class KnownPositionGyroscopeCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = true;

    /**
     * Indicates that by default G-dependent cross biases introduced
     * by the accelerometer on the gyroscope are estimated.
     */
    public static final boolean DEFAULT_ESTIMATE_G_DPENDENT_CROSS_BIASES = true;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope when G-dependent cross biases are being estimated.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES = 18;

    /**
     * Number of unknowns for the general case when G-dependent cross
     * biases are being estimated.
     */
    public static final int GENERAL_UNKNOWNS_AND_CROSS_BIASES = 21;

    /**
     * Number of unknowns when common z-axis is assumed for both
     * the accelerometer and gyroscope when G-dependent cross biases
     * are not being estimated.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS = 9;

    /**
     * Number of unknowns for the general case when G-dependent cross
     * biases are not being estimated.
     */
    public static final int GENERAL_UNKNOWNS = 12;

    /**
     * Required minimum number of sequences when common z-axis is assumed
     * and G-dependent cross biases are being estimated.
     */
    public static final int MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES =
            COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES + 1;

    /**
     * Required minimum number of sequences for the general case and
     * G-dependent cross biases are being estimated.
     */
    public static final int MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES =
            GENERAL_UNKNOWNS_AND_CROSS_BIASES + 1;

    /**
     * Required minimum number of sequences when common z-axis is assumed
     * and G-dependent cross biases are being ignored.
     */
    public static final int MINIMUM_SEQUENCES_COMMON_Z_AXIS =
            COMMON_Z_AXIS_UNKNOWNS + 1;

    /**
     * Required minimum number of sequences for the general case and
     * G-dependent cross biases are being ignored.
     */
    public static final int MINIMUM_SEQUENCES_GENERAL =
            GENERAL_UNKNOWNS + 1;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter mFitter =
            new LevenbergMarquardtMultiVariateFitter();

    /**
     * Known x-coordinate of accelerometer bias to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mAccelerometerBiasX;

    /**
     * Known y-coordinate of accelerometer bias to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mAccelerometerBiasY;

    /**
     * Known z-coordinate of accelerometer bias to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mAccelerometerBiasZ;

    /**
     * Known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerSx;

    /**
     * Known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerSy;

    /**
     * Known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerSz;

    /**
     * Known accelerometer x-y cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMxy;

    /**
     * Know accelerometer x-z cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMxz;

    /**
     * Known accelerometer y-x cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMyx;

    /**
     * Known accelerometer y-z cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMyz;

    /**
     * Known accelerometer z-x cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMzx;

    /**
     * Known accelerometer z-y cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMzy;

    /**
     * Initial x-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     */
    private double mInitialBiasX;

    /**
     * Initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     */
    private double mInitialBiasY;

    /**
     * Initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     */
    private double mInitialBiasZ;

    /**
     * Initial gyroscope x scaling factor.
     */
    private double mInitialSx;

    /**
     * Initial gyroscope y scaling factor.
     */
    private double mInitialSy;

    /**
     * Initial gyroscope z scaling factor.
     */
    private double mInitialSz;

    /**
     * Initial gyroscope x-y cross coupling error.
     */
    private double mInitialMxy;

    /**
     * Initial gyroscope x-z cross coupling error.
     */
    private double mInitialMxz;

    /**
     * Initial gyroscope y-x cross coupling error.
     */
    private double mInitialMyx;

    /**
     * Initial gyroscope y-z cross coupling error.
     */
    private double mInitialMyz;

    /**
     * Initial gyroscope z-x cross coupling error.
     */
    private double mInitialMzx;

    /**
     * Initial gyroscope z-y cross coupling error.
     */
    private double mInitialMzy;

    /**
     * Initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     */
    private Matrix mInitialGg;

    /**
     * Contains a collection of sequences of timestamped body kinematics
     * measurements taken at a given position where the device moves freely
     * with different orientations.
     */
    private Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mSequences;

    /**
     * Earth position where sequences of timestamped body kinematics
     * measurements were taken.
     */
    private ECEFPosition mPosition;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * This flag indicates whether G-dependent cross biases are being
     * estimated or not.
     * When enabled, this adds 9 variables from Gg matrix.
     */
    private boolean mEstimateGDependentCrossBiases =
            DEFAULT_ESTIMATE_G_DPENDENT_CROSS_BIASES;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownPositionGyroscopeCalibratorListener mListener;

    /**
     * Estimated angular rate biases for each IMU axis expressed in radians per
     * second (rad/s).
     */
    private double[] mEstimatedBiases;

    /**
     * Estimated gyroscope scale factors and cross coupling errors.
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
     */
    private Matrix mEstimatedMg;

    /**
     * Estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     */
    private Matrix mEstimatedGg;

    /**
     * Estimated covariance matrix for estimated parameters.
     */
    private Matrix mEstimatedCovariance;

    /**
     * Estimated chi square value.
     */
    private double mEstimatedChiSq;

    /**
     * Indicates whether calibrator is running.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public KnownPositionGyroscopeCalibrator() {
        try {
            mInitialGg = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must be 3x1 and is expressed in radians per
     *                    second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this();
        mPosition = position;
        mSequences = sequences;
        try {
            setInitialBias(initialBias);
            setInitialMg(initialMg);
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must be 3x1 and is expressed in radians per
     *                    second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, initialBias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a
     *                    solution. This must have length 3 and is expressed
     *                    in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this();
        mPosition = position;
        mSequences = sequences;
        try {
            setInitialBias(initialBias);
            setInitialMg(initialMg);
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a
     *                    solution. This must have length 3 and is expressed
     *                    in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, initialBias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, sequences, initialBias, initialMg, initialGg);
        try {
            setAccelerometerBias(accelerometerBias);
            setAccelerometerMa(accelerometerMa);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, sequences, initialBias, initialMg, initialGg);
        try {
            setAccelerometerBias(accelerometerBias);
            setAccelerometerMa(accelerometerMa);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(position, sequences, initialBias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, commonAxisUsed, estimateGDependentCrossBiases,
                initialBias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(position, sequences, initialBias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, commonAxisUsed, estimateGDependentCrossBiases,
                initialBias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must be 3x1 and is expressed in radians per
     *                    second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), sequences, initialBias, initialMg,
                initialGg);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must be 3x1 and is expressed in radians per
     *                    second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, initialBias, initialMg,
                initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a
     *                    solution. This must have length 3 and is expressed
     *                    in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), sequences, initialBias, initialMg,
                initialGg);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a
     *                    solution. This must have length 3 and is expressed
     *                    in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, initialBias, initialMg,
                initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics measures in the
     *                                      sequences have been taken.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
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
    public KnownPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix initialBias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known x-coordinate of accelerometer bias.
     */
    public double getAccelerometerBiasX() {
        return mAccelerometerBiasX;
    }

    /**
     * Sets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasX known x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerBiasX(final double accelerometerBiasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasX = accelerometerBiasX;
    }

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known y-coordinate of accelerometer bias.
     */
    public double getAccelerometerBiasY() {
        return mAccelerometerBiasY;
    }

    /**
     * Sets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasY known y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerBiasY(final double accelerometerBiasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasY = accelerometerBiasY;
    }

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known z-coordinate of accelerometer bias.
     */
    public double getAccelerometerBiasZ() {
        return mAccelerometerBiasZ;
    }

    /**
     * Sets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasZ known z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerBiasZ(final double accelerometerBiasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasZ = accelerometerBiasZ;
    }

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known x-coordinate of accelerometer bias.
     */
    public Acceleration getAccelerometerBiasXAsAcceleration() {
        return new Acceleration(mAccelerometerBiasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    public void getAccelerometerBiasXAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerometerBiasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerBiasX(final Acceleration accelerometerBiasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasX = convertAcceleration(accelerometerBiasX);
    }

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known y-coordinate of accelerometer bias.
     */
    public Acceleration getAccelerometerBiasYAsAcceleration() {
        return new Acceleration(mAccelerometerBiasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    public void getAccelerometerBiasYAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerometerBiasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerBiasY(final Acceleration accelerometerBiasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasY = convertAcceleration(accelerometerBiasY);
    }

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known z-coordinate of accelerometer bias.
     */
    public Acceleration getAccelerometerBiasZAsAcceleration() {
        return new Acceleration(mAccelerometerBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    public void getAccelerometerBiasZAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerometerBiasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerBiasZ(final Acceleration accelerometerBiasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasZ = convertAcceleration(accelerometerBiasZ);
    }

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerBias(
            final double accelerometerBiasX,
            final double accelerometerBiasY,
            final double accelerometerBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerBiasX = accelerometerBiasX;
        mAccelerometerBiasY = accelerometerBiasY;
        mAccelerometerBiasZ = accelerometerBiasZ;
    }

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerBias(
            final Acceleration accelerometerBiasX,
            final Acceleration accelerometerBiasY,
            final Acceleration accelerometerBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerBiasX = convertAcceleration(accelerometerBiasX);
        mAccelerometerBiasY = convertAcceleration(accelerometerBiasY);
        mAccelerometerBiasZ = convertAcceleration(accelerometerBiasZ);
    }

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known accelerometer bias.
     */
    public double[] getAccelerometerBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getAccelerometerBias(result);
        return result;
    }

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void getAccelerometerBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mAccelerometerBiasX;
        result[1] = mAccelerometerBiasY;
        result[2] = mAccelerometerBiasZ;
    }

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBias known accelerometer bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void setAccelerometerBias(final double[] accelerometerBias)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (accelerometerBias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mAccelerometerBiasX = accelerometerBias[0];
        mAccelerometerBiasY = accelerometerBias[1];
        mAccelerometerBiasZ = accelerometerBias[2];
    }

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known accelerometer bias.
     */
    public Matrix getAccelerometerBiasAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getAccelerometerBiasAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void getAccelerometerBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mAccelerometerBiasX);
        result.setElementAtIndex(1, mAccelerometerBiasY);
        result.setElementAtIndex(2, mAccelerometerBiasZ);
    }

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBias known accelerometer bias. Must be 3x1.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setAccelerometerBias(final Matrix accelerometerBias)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (accelerometerBias.getRows() != BodyKinematics.COMPONENTS
                || accelerometerBias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mAccelerometerBiasX = accelerometerBias.getElementAtIndex(0);
        mAccelerometerBiasY = accelerometerBias.getElementAtIndex(1);
        mAccelerometerBiasZ = accelerometerBias.getElementAtIndex(2);
    }

    /**
     * Gets known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer x scaling factor.
     */
    public double getAccelerometerSx() {
        return mAccelerometerSx;
    }

    /**
     * Sets known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSx known accelerometer x scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerSx(final double accelerometerSx)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerSx = accelerometerSx;
    }

    /**
     * Gets known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer y scaling factor.
     */
    public double getAccelerometerSy() {
        return mAccelerometerSy;
    }

    /**
     * Sets known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSy known accelerometer y scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerSy(final double accelerometerSy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerSy = accelerometerSy;
    }

    /**
     * Gets known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer z scaling factor.
     */
    public double getAccelerometerSz() {
        return mAccelerometerSz;
    }

    /**
     * Sets known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSz known accelerometer z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerSz(final double accelerometerSz)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerSz = accelerometerSz;
    }

    /**
     * Gets known accelerometer x-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer x-y cross coupling error.
     */
    public double getAccelerometerMxy() {
        return mAccelerometerMxy;
    }

    /**
     * Sets known accelerometer x-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxy known accelerometer x-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerMxy(final double accelerometerMxy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMxy = accelerometerMxy;
    }

    /**
     * Gets known accelerometer x-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer x-z cross coupling error.
     */
    public double getAccelerometerMxz() {
        return mAccelerometerMxz;
    }

    /**
     * Sets known accelerometer x-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxz known accelerometer x-z cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerMxz(final double accelerometerMxz)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMxz = accelerometerMxz;
    }

    /**
     * Gets known accelerometer y-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer y-x cross coupling error.
     */
    public double getAccelerometerMyx() {
        return mAccelerometerMyx;
    }

    /**
     * Sets known accelerometer y-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerMyx(final double accelerometerMyx)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMyx = accelerometerMyx;
    }

    /**
     * Gets known accelerometer y-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer y-z cross coupling error.
     */
    public double getAccelerometerMyz() {
        return mAccelerometerMyz;
    }

    /**
     * Sets known accelerometer y-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerMyz(final double accelerometerMyz)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMyz = accelerometerMyz;
    }

    /**
     * Gets known accelerometer z-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer z-x cross coupling error.
     */
    public double getAccelerometerMzx() {
        return mAccelerometerMzx;
    }

    /**
     * Sets known accelerometer z-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerMzx(final double accelerometerMzx)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMzx = accelerometerMzx;
    }

    /**
     * Gets known accelerometer z-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer z-y cross coupling error.
     */
    public double getAccelerometerMzy() {
        return mAccelerometerMzy;
    }

    /**
     * Sets known accelerometer z-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerMzy(final double accelerometerMzy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMzy = accelerometerMzy;
    }

    /**
     * Sets known accelerometer scaling factors to be used to fix measured
     * specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerSx known accelerometer x scaling factor.
     * @param accelerometerSy known accelerometer y scaling factor.
     * @param accelerometerSz known accelerometer z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerScalingFactors(
            final double accelerometerSx, final double accelerometerSy,
            final double accelerometerSz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerSx = accelerometerSx;
        mAccelerometerSy = accelerometerSy;
        mAccelerometerSz = accelerometerSz;
    }

    /**
     * Sets known accelerometer cross coupling errors to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxy known accelerometer x-y cross coupling
     *                         error.
     * @param accelerometerMxz known accelerometer x-z cross coupling
     *                         error.
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerCrossCouplingErrors(
            final double accelerometerMxy, final double accelerometerMxz,
            final double accelerometerMyx, final double accelerometerMyz,
            final double accelerometerMzx, final double accelerometerMzy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMxy = accelerometerMxy;
        mAccelerometerMxz = accelerometerMxz;
        mAccelerometerMyx = accelerometerMyx;
        mAccelerometerMyz = accelerometerMyz;
        mAccelerometerMzx = accelerometerMzx;
        mAccelerometerMzy = accelerometerMzy;
    }

    /**
     * Sets known accelerometer scaling factors and cross coupling errors
     * to be used to fix measured specific force and find cross biases
     * introduced by the accelerometer.
     *
     * @param accelerometerSx  known accelerometer x scaling factor.
     * @param accelerometerSy  known accelerometer y scaling factor.
     * @param accelerometerSz  known accelerometer z scaling factor.
     * @param accelerometerMxy known accelerometer x-y cross coupling
     *                         error.
     * @param accelerometerMxz known accelerometer x-z cross coupling
     *                         error.
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setAccelerometerScalingFactorsAndCrossCouplingErrors(
            final double accelerometerSx, final double accelerometerSy,
            final double accelerometerSz, final double accelerometerMxy,
            final double accelerometerMxz, final double accelerometerMyx,
            final double accelerometerMyz, final double accelerometerMzx,
            final double accelerometerMzy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        setAccelerometerScalingFactors(accelerometerSx, accelerometerSy,
                accelerometerSz);
        setAccelerometerCrossCouplingErrors(accelerometerMxy,
                accelerometerMxz, accelerometerMyx,
                accelerometerMyz, accelerometerMzx,
                accelerometerMzy);
    }

    /**
     * Gets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @return known accelerometer scale factors and cross coupling
     * errors matrix.
     */
    public Matrix getAccelerometerMa() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            getAccelerometerMa(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void getAccelerometerMa(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS ||
                result.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mAccelerometerSx);
        result.setElementAtIndex(1, mAccelerometerMyx);
        result.setElementAtIndex(2, mAccelerometerMzx);

        result.setElementAtIndex(3, mAccelerometerMxy);
        result.setElementAtIndex(4, mAccelerometerSy);
        result.setElementAtIndex(5, mAccelerometerMzy);

        result.setElementAtIndex(6, mAccelerometerMxz);
        result.setElementAtIndex(7, mAccelerometerMyz);
        result.setElementAtIndex(8, mAccelerometerSz);
    }

    /**
     * Sets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @param accelerometerMa known accelerometer scale factors and
     *                        cross coupling errors matrix. Must be 3x3.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setAccelerometerMa(final Matrix accelerometerMa)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (accelerometerMa.getRows() != BodyKinematics.COMPONENTS
                || accelerometerMa.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mAccelerometerSx = accelerometerMa.getElementAtIndex(0);
        mAccelerometerMyx = accelerometerMa.getElementAtIndex(1);
        mAccelerometerMzx = accelerometerMa.getElementAtIndex(2);

        mAccelerometerMxy = accelerometerMa.getElementAtIndex(3);
        mAccelerometerSy = accelerometerMa.getElementAtIndex(4);
        mAccelerometerMzy = accelerometerMa.getElementAtIndex(5);

        mAccelerometerMxz = accelerometerMa.getElementAtIndex(6);
        mAccelerometerMyz = accelerometerMa.getElementAtIndex(7);
        mAccelerometerSz = accelerometerMa.getElementAtIndex(8);
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    public double getInitialBiasX() {
        return mInitialBiasX;
    }

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasX(final double initialBiasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = initialBiasX;
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    public double getInitialBiasY() {
        return mInitialBiasY;
    }

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasY(final double initialBiasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasY = initialBiasY;
    }

    /**
     * Gets initial z-coordinate of gyroscope bias ot be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    public double getInitialBiasZ() {
        return mInitialBiasZ;
    }

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasZ(final double initialBiasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasZ = initialBiasZ;
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    public AngularSpeed getInitialBiasAngularSpeedX() {
        return new AngularSpeed(mInitialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(mInitialBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasX(final AngularSpeed initialBiasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = convertAngularSpeed(initialBiasX);
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    public AngularSpeed getInitialBiasAngularSpeedY() {
        return new AngularSpeed(mInitialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(mInitialBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasY(final AngularSpeed initialBiasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasY = convertAngularSpeed(initialBiasY);
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    public AngularSpeed getInitialBiasAngularSpeedZ() {
        return new AngularSpeed(mInitialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mInitialBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param inigialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasZ(final AngularSpeed inigialBiasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasZ = convertAngularSpeed(inigialBiasZ);
    }

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution
     * expressed in radians per second (rad/s).
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBias(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = initialBiasX;
        mInitialBiasY = initialBiasY;
        mInitialBiasZ = initialBiasZ;
    }

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBias(
            final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = convertAngularSpeed(initialBiasX);
        mInitialBiasY = convertAngularSpeed(initialBiasY);
        mInitialBiasZ = convertAngularSpeed(initialBiasZ);
    }

    /**
     * Gets initial x scaling factor of gyroscope.
     *
     * @return initial x scaling factor of gyroscope.
     */
    public double getInitialSx() {
        return mInitialSx;
    }

    /**
     * Sets initial x scaling factor of gyroscope.
     *
     * @param initialSx initial x scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialSx(final double initialSx)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSx = initialSx;
    }

    /**
     * Gets initial y scaling factor of gyroscope.
     *
     * @return initial y scaling factor of gyroscope.
     */
    public double getInitialSy() {
        return mInitialSy;
    }

    /**
     * Sets initial y scaling factor of gyroscope.
     *
     * @param initialSy initial y scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialSy(final double initialSy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSy = initialSy;
    }

    /**
     * Gets initial z scaling factor of gyroscope.
     *
     * @return initial z scaling factor of gyroscope.
     */
    public double getInitialSz() {
        return mInitialSz;
    }

    /**
     * Sets initial z scaling factor of gyroscope.
     *
     * @param initialSz initial z scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialSz(final double initialSz)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSz = initialSz;
    }

    /**
     * Gets initial x-y cross coupling error of gyroscope.
     *
     * @return initial x-y cross coupling error of gyroscope.
     */
    public double getInitialMxy() {
        return mInitialMxy;
    }

    /**
     * Sets initial x-y cross coupling error of gyroscope.
     *
     * @param initialMxy initial x-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMxy(final double initialMxy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMxy = initialMxy;
    }

    /**
     * Gets initial x-z cross coupling error of gyroscope.
     *
     * @return initial x-z cross coupling error of gyroscope.
     */
    public double getInitialMxz() {
        return mInitialMxz;
    }

    /**
     * Sets initial x-z cross coupling error of gyroscope.
     *
     * @param initialMxz initial x-z cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMxz(final double initialMxz)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMxz = initialMxz;
    }

    /**
     * Gets initial y-x cross coupling error of gyroscope.
     *
     * @return initial y-x cross coupling error of gyroscope.
     */
    public double getInitialMyx() {
        return mInitialMyx;
    }

    /**
     * Sets initial y-x cross coupling error of gyroscope.
     *
     * @param initialMyx initial y-x cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMyx(final double initialMyx)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMyx = initialMyx;
    }

    /**
     * Gets initial y-z cross coupling error of gyroscope.
     *
     * @return initial y-z cross coupling error of gyroscope.
     */
    public double getInitialMyz() {
        return mInitialMyz;
    }

    /**
     * Sets initial y-z cross coupling error of gyroscope.
     *
     * @param initialMyz initial y-z cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMyz(final double initialMyz)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMyz = initialMyz;
    }

    /**
     * Gets initial z-x cross coupling error of gyroscope.
     *
     * @return initial z-x cross coupling error of gyroscope.
     */
    public double getInitialMzx() {
        return mInitialMzx;
    }

    /**
     * Sets initial z-x cross coupling error of gyroscope.
     *
     * @param initialMzx initial z-x cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMzx(final double initialMzx)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMzx = initialMzx;
    }

    /**
     * Gets initial z-y cross coupling error of gyroscope.
     *
     * @return initial z-y cross coupling error of gyroscope.
     */
    public double getInitialMzy() {
        return mInitialMzy;
    }

    /**
     * Sets initial z-y cross coupling error of gyroscope.
     *
     * @param initialMzy initial z-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMzy(final double initialMzy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMzy = initialMzy;
    }

    /**
     * Sets initial scaling factors of gyroscope.
     *
     * @param initialSx initial x scaling factor of gyroscope.
     * @param initialSy initial y scaling factor of gyroscope.
     * @param initialSz initial z scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialScalingFactors(
            final double initialSx, final double initialSy,
            final double initialSz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSx = initialSx;
        mInitialSy = initialSy;
        mInitialSz = initialSz;
    }

    /**
     * Sets initial cross coupling errors of gyroscope.
     *
     * @param initialMxy initial x-y cross coupling error of gyroscope.
     * @param initialMxz initial x-z cross coupling error of gyroscope.
     * @param initialMyx initial y-x cross coupling error of gyroscope.
     * @param initialMyz initial y-z cross coupling error of gyroscope.
     * @param initialMzx initial z-x cross coupling error of gyroscope.
     * @param initialMzy initial z-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialCrossCouplingErrors(
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMxy = initialMxy;
        mInitialMxz = initialMxz;
        mInitialMyx = initialMyx;
        mInitialMyz = initialMyz;
        mInitialMzx = initialMzx;
        mInitialMzy = initialMzy;
    }

    /**
     * Sets initial scaling factors and cross coupling errors of
     * gyroscope.
     *
     * @param initialSx  initial x scaling factor of gyroscope.
     * @param initialSy  initial y scaling factor of gyroscope.
     * @param initialSz  initial z scaling factor of gyroscope.
     * @param initialMxy initial x-y cross coupling error of gyroscope.
     * @param initialMxz initial x-z cross coupling error of gyroscope.
     * @param initialMyx initial y-x cross coupling error of gyroscope.
     * @param initialMyz initial y-z cross coupling error of gyroscope.
     * @param initialMzx initial z-x cross coupling error of gyroscope.
     * @param initialMzy initial z-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialScalingFactorsAndCrossCouplingErrors(
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        setInitialScalingFactors(initialSx, initialSy, initialSz);
        setInitialCrossCouplingErrors(initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Gets initial gyroscope bias to be used to find a solution as
     * an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of initial gyroscope bias.
     */
    public double[] getInitialBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getInitialBias(result);
        return result;
    }

    /**
     * Gets initial gyroscope bias to be used to find a solution as
     * an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getInitialBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = mInitialBiasX;
        result[1] = mInitialBiasY;
        result[2] = mInitialBiasZ;
    }

    /**
     * Sets initial gyroscope bias to be used to find a solution as
     * an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setInitialBias(final double[] initialBias)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (initialBias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        mInitialBiasX = initialBias[0];
        mInitialBiasY = initialBias[1];
        mInitialBiasZ = initialBias[2];
    }

    /**
     * Gets initial gyroscope bias to be used to find a solution as a
     * column matrix.
     *
     * @return initial gyroscope bias to be used to find a solution as a
     * column matrix.
     */
    public Matrix getInitialBiasAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getInitialBiasAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets initial gyroscope bias to be used to find a solution as a
     * column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void getInitialBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mInitialBiasX);
        result.setElementAtIndex(1, mInitialBiasY);
        result.setElementAtIndex(2, mInitialBiasZ);
    }

    /**
     * Sets initial gyroscope bias to be used to find a solution as
     * an array.
     *
     * @param initialBias initial gyroscope bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setInitialBias(final Matrix initialBias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialBias.getRows() != BodyKinematics.COMPONENTS
                || initialBias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mInitialBiasX = initialBias.getElementAtIndex(0);
        mInitialBiasY = initialBias.getElementAtIndex(1);
        mInitialBiasZ = initialBias.getElementAtIndex(2);
    }

    /**
     * Gets initial gyroscope scale factors and cross coupling errors
     * matrix.
     *
     * @return initial gyroscope scale factors and cross coupling errors
     * matrix.
     */
    public Matrix getInitialMg() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            getInitialMg(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets initial gyroscope scale factors and cross coupling errors
     * matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void getInitialMg(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS ||
                result.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mInitialSx);
        result.setElementAtIndex(1, mInitialMyx);
        result.setElementAtIndex(2, mInitialMzx);

        result.setElementAtIndex(3, mInitialMxy);
        result.setElementAtIndex(4, mInitialSy);
        result.setElementAtIndex(5, mInitialMzy);

        result.setElementAtIndex(6, mInitialMxz);
        result.setElementAtIndex(7, mInitialMyz);
        result.setElementAtIndex(8, mInitialSz);
    }

    /**
     * Sets initial gyroscope scale factors and cross coupling errors matrix.
     *
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setInitialMg(final Matrix initialMg) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialMg.getRows() != BodyKinematics.COMPONENTS ||
                initialMg.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mInitialSx = initialMg.getElementAtIndex(0);
        mInitialMyx = initialMg.getElementAtIndex(1);
        mInitialMzx = initialMg.getElementAtIndex(2);

        mInitialMxy = initialMg.getElementAtIndex(3);
        mInitialSy = initialMg.getElementAtIndex(4);
        mInitialMzy = initialMg.getElementAtIndex(5);

        mInitialMxz = initialMg.getElementAtIndex(6);
        mInitialMyz = initialMg.getElementAtIndex(7);
        mInitialSz = initialMg.getElementAtIndex(8);
    }

    /**
     * Gets initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @return a 3x3 matrix containing initial g-dependent cross biases.
     */
    public Matrix getInitialGg() {
        return new Matrix(mInitialGg);
    }

    /**
     * Gets initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void getInitialGg(final Matrix result) {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result.copyFrom(mInitialGg);
    }

    /**
     * Sets initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @param initialGg g-dependent cross biases.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setInitialGg(final Matrix initialGg) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (initialGg.getRows() != BodyKinematics.COMPONENTS
                || initialGg.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        initialGg.copyTo(mInitialGg);
    }

    /**
     * Gets collection of sequences of timestamped body kinematics
     * measurements taken at a given position where the device moves freely
     * with different orientations.
     *
     * @return collection of sequences of timestamped body kinematics
     * measurements.
     */
    public Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> getSequences() {
        return mSequences;
    }

    /**
     * Sets collection of sequences of timestamped body kinematics
     * measurements taken at a given position where the device moves freely
     * with different orientations.
     *
     * @param sequences collection of sequences of timestamped body
     *                  kinematics measurements.
     * @throws LockedException if calibrator is currently running.
     */
    public void setSequences(
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mSequences = sequences;
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
        if (mRunning) {
            throw new LockedException();
        }

        mPosition = position;
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
        if (mRunning) {
            throw new LockedException();
        }

        mPosition = convertPosition(position);
    }

    /**
     * Indicates whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @return true if z-axis is assumed to be common for accelerometer and gyroscope,
     * false otherwise.
     */
    public boolean isCommonAxisUsed() {
        return mCommonAxisUsed;
    }

    /**
     * Specifies whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @param commonAxisUsed true if z-axis is assumed to be common for accelerometer
     *                       and gyroscope, false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    public void setCommonAxisUsed(final boolean commonAxisUsed) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Indicates whether G-dependent cross biases are being estimated
     * or not.
     * When enabled, this adds 9 variables from Gg matrix.
     *
     * @return true if G-dependent cross biases will be estimated,
     * false otherwise.
     */
    public boolean isGDependentCrossBiasesEstimated() {
        return mEstimateGDependentCrossBiases;
    }

    /**
     * Specifies whether G-dependent cross biases are being estimated
     * or not.
     * When enabled, this adds 9 variables from Gg matrix.
     *
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated,
     *                                      false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    public void setGDependentCrossBiasesEstimated(
            final boolean estimateGDependentCrossBiases)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public KnownPositionGyroscopeCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final KnownPositionGyroscopeCalibratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets minimum number of required sequences.
     *
     * @return minimum number of required sequences.
     */
    public int getMinimumRequiredSequences() {
        if (mCommonAxisUsed) {
            if (mEstimateGDependentCrossBiases) {
                return MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            } else {
                return MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            }
        } else {
            if (mEstimateGDependentCrossBiases) {
                return MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES;
            } else {
                return MINIMUM_SEQUENCES_GENERAL;
            }
        }
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    public boolean isReady() {
        return mSequences != null
                && mSequences.size() >= getMinimumRequiredSequences();
    }

    /**
     * Indicates whether calibrator is currently running or not.
     *
     * @return true if calibrator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Estimates gyroscope calibration parameters containing bias, scale factors,
     * cross-coupling errors and G-dependent coupling.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (mRunning) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onCalibrateStart(this);
            }

            if (mCommonAxisUsed) {
                if (mEstimateGDependentCrossBiases) {
                    calibrateCommonAxisAndGDependentCrossBiases();
                } else {
                    calibrateCommonAxis();
                }
            } else {
                if (mEstimateGDependentCrossBiases) {
                    calibrateGeneralAndGDependentCrossBiases();
                } else {
                    calibrateGeneral();
                }
            }

            if (mListener != null) {
                mListener.onCalibrateEnd(this);
            }
//        } catch (final FittingException e) {
//            throw new CalibrationException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return array containing x,y,z components of estimated gyroscope biases.
     */
    public double[] getEstimatedBiases() {
        return mEstimatedBiases;
    }

    /**
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @param result instance where estimated gyroscope biases will be stored.
     * @return true if result instance was updated, false otherwise (when estimation
     * is not yet available).
     */
    public boolean getEstimatedBiases(double[] result) {
        if (mEstimatedBiases != null) {
            System.arraycopy(mEstimatedBiases, 0, result,
                    0, mEstimatedBiases.length);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets column matrix containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return column matrix containing x,y,z components of estimated gyroscope
     * biases.
     */
    public Matrix getEstimatedBiasesAsMatrix() {
        return mEstimatedBiases != null ? Matrix.newFromArray(mEstimatedBiases) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    public boolean getEstimatedBiasesAsMatrix(Matrix result)
            throws WrongSizeException {
        if (mEstimatedBiases != null) {
            result.fromArray(mEstimatedBiases);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets x coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasX() {
        return mEstimatedBiases != null ? mEstimatedBiases[0] : null;
    }

    /**
     * Gets y coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasY() {
        return mEstimatedBiases != null ? mEstimatedBiases[1] : null;
    }

    /**
     * Gets z coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return z coordinate of estimated gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasZ() {
        return mEstimatedBiases != null ? mEstimatedBiases[2] : null;
    }

    /**
     * Gets x coordinate of estimated gyroscope bias.
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    public AngularSpeed getEstimatedBiasAngularSpeedX() {
        return mEstimatedBiases != null ?
                new AngularSpeed(mEstimatedBiases[0],
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets x coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    public boolean getEstimatedBiasAngularSpeedX(AngularSpeed result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[0]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets y coordinate of estimated gyroscope bias.
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    public AngularSpeed getEstimatedBiasAngularSpeedY() {
        return mEstimatedBiases != null ?
                new AngularSpeed(mEstimatedBiases[1],
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets y coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    public boolean getEstimatedBiasAngularSpeedY(AngularSpeed result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[1]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets z coordinate of estimated gyroscope bias.
     *
     * @return z coordinate of estimated gyroscope bias or null if not available.
     */
    public AngularSpeed getEstimatedBiasAngularSpeedZ() {
        return mEstimatedBiases != null ?
                new AngularSpeed(mEstimatedBiases[2],
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets z coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    public boolean getEstimatedBiasAngularSpeedZ(AngularSpeed result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[2]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

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
     * @return estimated gyroscope scale factors and cross coupling errors, or null
     * if not available.
     */
    public Matrix getEstimatedMg() {
        return mEstimatedMg;
    }

    /**
     * Gets estimated gyroscope x-axis scale factor.
     *
     * @return estimated gyroscope x-axis scale factor or null
     * if not available.
     */
    public Double getEstimatedSx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated gyroscope y-axis scale factor.
     *
     * @return estimated gyroscope y-axis scale factor or null
     * if not available.
     */
    public Double getEstimatedSy() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated gyroscope z-axis scale factor.
     *
     * @return estimated gyroscope z-axis scale factor or null
     * if not available.
     */
    public Double getEstimatedSz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated gyroscope x-y cross-coupling error.
     *
     * @return estimated gyroscope x-y cross-coupling error or null
     * if not available.
     */
    public Double getEstimatedMxy() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated gyroscope x-z cross-coupling error.
     *
     * @return estimated gyroscope x-z cross-coupling error or null
     * if not available.
     */
    public Double getEstimatedMxz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated gyroscope y-x cross-coupling error.
     *
     * @return estimated gyroscope y-x cross-coupling error or null
     * if not available.
     */
    public Double getEstimatedMyx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated gyroscope y-z cross-coupling error.
     *
     * @return estimated gyroscope y-z cross-coupling error or null
     * if not available.
     */
    public Double getEstimatedMyz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated gyroscope z-x cross-coupling error.
     *
     * @return estimated gyroscope z-x cross-coupling error or null
     * if not available.
     */
    public Double getEstimatedMzx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated gyroscope z-y cross-coupling error.
     *
     * @return estimated gyroscope z-y cross-coupling error or null
     * if not available.
     */
    public Double getEstimatedMzy() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(2, 1) : null;
    }

    /**
     * Gets estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     *
     * @return estimated G-dependent cross biases.
     */
    public Matrix getEstimatedGg() {
        return mEstimatedGg;
    }

    /**
     * Gets estimated covariance matrix for estimated parameters.
     *
     * @return estimated covariance matrix for estimated parameters.
     */
    public Matrix getEstimatedCovariance() {
        return mEstimatedCovariance;
    }

    /**
     * Gets estimated chi square value.
     *
     * @return estimated chi square value.
     */
    public double getEstimatedChiSq() {
        return mEstimatedChiSq;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed
     * for both the accelerometer and gyroscope and when G-dependent cross
     * biases are being estimated.
     */
    private void calibrateCommonAxisAndGDependentCrossBiases() {
// TODO: finish
    }

    /**
     * Internal method to perform general calibration when G-dependent cross
     * biases are being estimated.
     */
    private void calibrateGeneralAndGDependentCrossBiases() {
// TODO: finish
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed
     * for both the accelerometer and gyroscope and G-dependent cross biases
     * are ignored.
     */
    private void calibrateCommonAxis() {
// TODO: finish
    }

    /**
     * Internal method to perform general calibration when G-dependent cross
     * biases are ignored.
     */
    private void calibrateGeneral() {
// TODO: finish
    }

    private void setInputData() throws AlgebraException, InvalidRotationMatrixException,
            InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {

        int numMeasurements = 0;
        for (BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence : mSequences) {
            final int count = sequence.getItemsCount();
            if (count > 0) {
                numMeasurements += (sequence.getItemsCount() - 1);
            }
        }

        final double[] measuredF = new double[BodyKinematics.COMPONENTS];
        final double[] fixedF = new double[BodyKinematics.COMPONENTS];
        final Matrix ba = getAccelerometerBiasAsMatrix();
        final Matrix ma = getAccelerometerMa();
        final Matrix x = new Matrix(numMeasurements, 2 * BodyKinematics.COMPONENTS);
        final Matrix y = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final double[] standardDeviations = new double[numMeasurements];

        final ECEFFrame frame = new ECEFFrame();
        final double posX = mPosition.getX();
        final double posY = mPosition.getY();
        final double posZ = mPosition.getZ();
        final ECEFGravity gravity = new ECEFGravity();
        final double[] normF = new double[BodyKinematics.COMPONENTS];
        final double[] normG = new double[BodyKinematics.COMPONENTS];
        final Matrix skew = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        final Matrix tmp1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final Matrix tmp2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final double[] axis = new double[BodyKinematics.COMPONENTS];

        final CoordinateTransformation oldC = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix rot = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);

        int i = 0;
        for (final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence : mSequences) {
            final List<StandardDeviationTimedBodyKinematics> sortedMeasurements =
                    sequence.getSortedItems();
            if (sortedMeasurements.isEmpty()) {
                continue;
            }

            boolean first = true;
            double previousTimestamp = 0.0;
            double timestamp;
            for (final StandardDeviationTimedBodyKinematics measurement : sortedMeasurements) {
                if (first) {
                    previousTimestamp = measurement.getTimestampSeconds();
                    first = false;

                    final BodyKinematics kinematics = measurement.getKinematics();
                    final double fx = kinematics.getFx();
                    final double fy = kinematics.getFy();
                    final double fz = kinematics.getFz();

                    // get normalized vector from measured specific force
                    normF[0] = fx;
                    normF[1] = fy;
                    normF[2] = fz;
                    ArrayUtils.normalize(normF);

                    // get normalized vector from gravity at current position
                    ECEFGravityEstimator.estimateGravity(posX, posY, posZ, gravity);
                    gravity.asArray(normG);
                    ArrayUtils.normalize(normG);

                    // compute angle between both normalized vectors using dot product
                    // cos(alpha) = normF' * normG
                    final double cosAlpha = ArrayUtils.dotProduct(normF, normG);
                    final double alpha = Math.acos(cosAlpha);

                    // compute vector perpendicular to both normF and normG which will
                    // be the rotation axis
                    Utils.skewMatrix(normG, skew);
                    tmp1.fromArray(normF);
                    skew.multiply(tmp1, tmp2);

                    tmp2.toArray(axis);

                    Quaternion q = new Quaternion(axis, alpha);
                    q.asInhomogeneousMatrix(rot);
                    oldC.setMatrix(rot);

                    // TODO: compute initial orientation oldC
                    continue;
                }

                timestamp = measurement.getTimestampSeconds();
                final double timeInterval = timestamp - previousTimestamp;
                previousTimestamp = timestamp;

                final BodyKinematics kinematics = measurement.getKinematics();
                measuredF[0] = kinematics.getFx();
                measuredF[1] = kinematics.getFy();
                measuredF[2] = kinematics.getFz();
                AccelerationFixer.fix(measuredF, ba, ma, fixedF);

                final double fx = fixedF[0];
                final double fy = fixedF[1];
                final double fz = fixedF[2];
                final double wx = kinematics.getAngularRateX();
                final double wy = kinematics.getAngularRateY();
                final double wz = kinematics.getAngularRateZ();

                x.setElementAt(i, 0, fx);
                x.setElementAt(i, 1, fy);
                x.setElementAt(i, 2, fz);
                x.setElementAt(i, 3, wx);
                x.setElementAt(i, 4, wy);
                x.setElementAt(i, 5, wz);

                ECEFInertialNavigator.navigateECEF(timeInterval,
                        posX, posY, posZ, oldC,
                        0.0, 0.0, 0.0,
                        fx, fy, fz, wx, wy, wz, frame);

                // TODO: Store orientation vector coordinates
                //y.setElementAt(i);

                standardDeviations[i] =
                        measurement.getAngularRateStandardDeviation();

                i++;
            }
        }

        //mFitter.setInputData();
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

    /**
     * Converts acceleration instance to meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(),
                acceleration.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts angular speed instance to radians per second.
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return AngularSpeedConverter.convert(angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }
}
