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
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.AccelerationFixer;
import com.irurueta.navigation.inertial.calibration.AngularRateFixer;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.navigation.inertial.estimators.LevelingEstimator;
import com.irurueta.navigation.inertial.estimators.LevelingEstimator2;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.JacobianEstimator;
import com.irurueta.numerical.MultiVariateFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.Collection;
import java.util.List;

/**
 * Estimates gyroscope cross couplings, scaling factors and
 * G-dependent cross biases introduced on the gyroscope by the specific
 * forces sensed by the accelerometer.
 * <p>
 * This calibrator assumes that the IMU is at a more or less fixed location on
 * Earth (to account for gravitational effects), and evaluates sequences of
 * measured body kinematics to perform calibration for unknown orientations
 * on those provided sequences.
 * <p>
 * To use this calibrator at least 7 sequences are needed (each containing
 * at least 3 body kinematics measurements) when common z-axis is assumed and
 * G-dependant cross biases are ignored, otherwise at least 10 sequences are
 * required (each containing at least 3 body kinematics measurements) when
 * common z-axis is not assumed.
 * If G-dependent cross biases are being estimated, then at least 16
 * measurements are needed when common z-axis is assumed, otherwise at least
 * 19 sequences are required (each containing at least 3 body kinematics
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
 * <p>
 * This calibrator is based on the ideas of
 * David Tedaldi, Alberto Pretto, Emmanuelle Menegatti. A Robust and Easy to
 * Implement Method for IMU Calibration without External Equipments.
 */
public class KnownBiasAndPositionGyroscopeCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = true;

    /**
     * Indicates that by default G-dependent cross biases introduced
     * by the accelerometer on the gyroscope are estimated.
     */
    public static final boolean DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES = true;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope when G-dependent cross biases are being estimated.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES = 15;

    /**
     * Number of unknowns for the general case when G-dependent cross
     * biases are being estimated.
     */
    public static final int GENERAL_UNKNOWNS_AND_CROSS_BIASES = 18;

    /**
     * Number of unknowns when common z-axis is assumed for both
     * the accelerometer and gyroscope when G-dependent cross biases
     * are not being estimated.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS = 6;

    /**
     * Number of unknowns for the general case when G-dependent cross
     * biases are not being estimated.
     */
    public static final int GENERAL_UNKNOWNS = 9;

    /**
     * Required minimum number of measurements when common z-axis is assumed
     * and G-dependent cross biases are being estimated.
     */
    public static final int MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES =
            COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES + 1;

    /**
     * Required minimum number of measurements for the general case and
     * G-dependent cross biases are being estimated.
     */
    public static final int MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES =
            GENERAL_UNKNOWNS_AND_CROSS_BIASES + 1;

    /**
     * Required minimum number of measurements when common z-axis is assumed
     * and G-dependent cross biases are being ignored.
     */
    public static final int MINIMUM_SEQUENCES_COMMON_Z_AXIS =
            COMMON_Z_AXIS_UNKNOWNS + 1;

    /**
     * Required minimum number of measurements for the general case and
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
     * Known x-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     */
    private double mBiasX;

    /**
     * Known y-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     */
    private double mBiasY;

    /**
     * Known z-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     */
    private double mBiasZ;

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
            DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownBiasAndPositionGyroscopeCalibratorListener mListener;

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
     * Internally holds x-coordinate of measured angular rate during calibration.
     */
    private double mMeasAngularRateX;

    /**
     * Internally holds y-coordinate of measured angular rate during calibration.
     */
    private double mMeasAngularRateY;

    /**
     * Internally holds z-coordinate of measured angular rate during calibration.
     */
    private double mMeasAngularRateZ;

    /**
     * Internally holds x-coordinate of measured and fixed specific force during calibration.
     */
    private double mFtrueX;

    /**
     * Internally holds y-coordinate of measured and fixed specific force during calibration.
     */
    private double mFtrueY;

    /**
     * Internally holds z-coordinate of measured and fixed specific force during calibration.
     */
    private double mFtrueZ;

    /**
     * Internally holds measured angular rate during calibration expressed as
     * a column matrix.
     */
    private Matrix mMeasAngularRate;

    /**
     * Internally holds cross-coupling errors during calibration.
     */
    private Matrix mM;

    /**
     * Internally holds inverse of cross-coupling errors during calibration.
     */
    private Matrix mInvM;

    /**
     * Internally holds biases during calibration.
     */
    private Matrix mB;

    /**
     * Internally hold g-dependent cross biases during calibration.
     */
    private Matrix mG;

    /**
     * Internally holds computed true angular rate during calibration.
     */
    private Matrix mTrueAngularRate;

    /**
     * Internally holds computed true specific force during calibration.
     */
    private Matrix mFtrue;

    /**
     * Internally holds angular rate bias due to g-dependent cross biases
     */
    private Matrix mTmp;

    /**
     * Internal coordinate transformation resolved around ECEF frame to be reused.
     */
    private CoordinateTransformation mOldC;

    /**
     * Internal ECEF frame to be reused.
     */
    private ECEFFrame mEcefFrame;

    /**
     * Acceleration fixer.
     */
    private final AccelerationFixer mAccelerationFixer =
            new AccelerationFixer();

    /**
     * Angular rate fixer.
     */
    private final AngularRateFixer mAngularRateFixer =
            new AngularRateFixer();

    /**
     * Navigation parameters to be used during evaluation.
     */
    private Matrix mNavigationParams;

    /**
     * ECEF frame to estimate gravity.
     */
    private final ECEFFrame mGravityFrame = new ECEFFrame();

    /**
     * ECEF position to estimate gravity.
     */
    private final ECEFPosition mGravityPosition = new ECEFPosition();

    /**
     * Attitude to estimate gravity.
     */
    private final CoordinateTransformation mGravityC =
            new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

    /**
     * Body kinematics to estimate gravity.
     */
    private final BodyKinematics mGravityKinematics = new BodyKinematics();

    /**
     * Estimated gravity.
     */
    private final double[] mGravity = new double[BodyKinematics.COMPONENTS];

    /**
     * Index of current point being evaluated.
     */
    private int mI;

    /**
     * Constructor.
     */
    public KnownBiasAndPositionGyroscopeCalibrator() {
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
     * @param bias initial gyroscope bias to be used to find a solution.
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this();
        mPosition = position;
        mSequences = sequences;
        try {
            setBias(bias);
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
     * @param bias initial gyroscope bias to be used to find a solution.
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, bias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param bias initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this();
        mPosition = position;
        mSequences = sequences;
        try {
            setBias(bias);
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
     * @param bias initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, bias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias       initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, sequences, bias, initialMg, initialGg);
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
     * @param bias       initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, bias, initialMg, initialGg,
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
     * @param bias       initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, sequences, bias, initialMg, initialGg);
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
     * @param bias       initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, bias, initialMg, initialGg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(position, sequences, bias, initialMg, initialGg);
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(position, sequences, bias, initialMg, initialGg);
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, sequences, bias, initialMg, initialGg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, sequences, bias, initialMg, initialGg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final ECEFPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(position, sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias initial gyroscope bias to be used to find a solution.
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), sequences, bias, initialMg,
                initialGg);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param bias initial gyroscope bias to be used to find a solution.
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, bias, initialMg,
                initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param bias initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), sequences, bias, initialMg,
                initialGg);
    }

    /**
     * Constructor.
     *
     * @param position    position where body kinematics measures in the
     *                    sequences have been taken.
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param bias initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, bias, initialMg,
                initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias       initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), sequences, bias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias       initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, bias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias       initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), sequences, bias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position          position where body kinematics measures in the
     *                          sequences have been taken.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias       initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * @param bias                   initial gyroscope bias to be used to find a
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
    public KnownBiasAndPositionGyroscopeCalibrator(
            final NEDPosition position,
            final Collection<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasAndPositionGyroscopeCalibratorListener listener) {
        this(convertPosition(position), sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
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
     * Gets known x-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return known x-coordinate of gyroscope bias.
     */
    public double getBiasX() {
        return mBiasX;
    }

    /**
     * Sets known x-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasX known x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasX(final double biasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
    }

    /**
     * Gets known y-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return known y-coordinate of gyroscope bias.
     */
    public double getBiasY() {
        return mBiasY;
    }

    /**
     * Sets known y-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasY known y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasY(final double biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = biasY;
    }

    /**
     * Gets known z-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return known z-coordinate of gyroscope bias.
     */
    public double getBiasZ() {
        return mBiasZ;
    }

    /**
     * Sets known z-coordinate of gyroscope bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasZ known z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasZ(final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = biasZ;
    }

    /**
     * Gets known x-coordinate of gyroscope bias.
     *
     * @return known x-coordinate of gyroscope bias.
     */
    public AngularSpeed getBiasAngularSpeedX() {
        return new AngularSpeed(mBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known x-coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    public void getBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(mBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known x-coordinate of gyroscope bias.
     *
     * @param biasX known x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasX(final AngularSpeed biasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = convertAngularSpeed(biasX);
    }

    /**
     * Gets known y-coordinate of gyroscope bias.
     *
     * @return known y-coordinate of gyroscope bias.
     */
    public AngularSpeed getBiasAngularSpeedY() {
        return new AngularSpeed(mBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known y-coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    public void getBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(mBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known y-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param biasY known y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasY(final AngularSpeed biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = convertAngularSpeed(biasY);
    }

    /**
     * Gets known z-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @return known z-coordinate of gyroscope bias.
     */
    public AngularSpeed getBiasAngularSpeedZ() {
        return new AngularSpeed(mBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known z-coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    public void getBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known z-coordinate of gyroscope bias.
     *
     * @param biasZ known z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasZ(final AngularSpeed biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Sets known bias coordinates of gyroscope expressed in radians
     * per second (rad/s).
     *
     * @param biasX known x-coordinate of gyroscope bias.
     * @param biasY known y-coordinate of gyroscope bias.
     * @param biasZ known z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(
            final double biasX, final double biasY,
            final double biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
        mBiasY = biasY;
        mBiasZ = biasZ;
    }

    /**
     * Sets known bias coordinates of gyroscope.
     *
     * @param biasX known x-coordinate of gyroscope bias.
     * @param biasY known y-coordinate of gyroscope bias.
     * @param biasZ known z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(
            final AngularSpeed biasX,
            final AngularSpeed biasY,
            final AngularSpeed biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = convertAngularSpeed(biasX);
        mBiasY = convertAngularSpeed(biasY);
        mBiasZ = convertAngularSpeed(biasZ);
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
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of gyroscope bias.
     */
    public double[] getBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = mBiasX;
        result[1] = mBiasY;
        result[2] = mBiasZ;
    }

    /**
     * Sets known gyroscope bias  as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setBias(final double[] bias)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (bias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        mBiasX = bias[0];
        mBiasY = bias[1];
        mBiasZ = bias[2];
    }

    /**
     * Gets known gyroscope bias as a column matrix.
     *
     * @return known gyroscope bias as a column matrix.
     */
    public Matrix getBiasAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getBiasAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known gyroscope bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void getBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mBiasX);
        result.setElementAtIndex(1, mBiasY);
        result.setElementAtIndex(2, mBiasZ);
    }

    /**
     * Sets known gyroscope bias as an array.
     *
     * @param bias known gyroscope bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setBias(final Matrix bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (bias.getRows() != BodyKinematics.COMPONENTS
                || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mBiasX = bias.getElementAtIndex(0);
        mBiasY = bias.getElementAtIndex(1);
        mBiasZ = bias.getElementAtIndex(2);
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
    public KnownBiasAndPositionGyroscopeCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final KnownBiasAndPositionGyroscopeCalibratorListener listener)
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
                && mSequences.size() >= getMinimumRequiredSequences()
                && mPosition != null;
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
    public void calibrate() throws LockedException, NotReadyException,
            CalibrationException {
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
        } catch (final FittingException
                | InertialNavigatorException
                | AlgebraException
                | InvalidSourceAndDestinationFrameTypeException
                | InvalidRotationMatrixException
                | com.irurueta.numerical.NotReadyException e) {
            throw new CalibrationException(e);
        } finally {
            mRunning = false;
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
     *
     * @throws FittingException                              if no convergence to solution is found.
     * @throws InertialNavigatorException                    if inertial navigation fails due to numerical
     *                                                       instabilities.
     * @throws AlgebraException                              if accelerometer parameters prevents fixing
     *                                                       measured accelerometer values due to numerical
     *                                                       instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws com.irurueta.numerical.NotReadyException      if fitter is not ready.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private void calibrateCommonAxisAndGDependentCrossBiases()
            throws FittingException,
            InertialNavigatorException,
            AlgebraException,
            InvalidSourceAndDestinationFrameTypeException,
            com.irurueta.numerical.NotReadyException,
            InvalidRotationMatrixException {
        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Where Mg is upper triangular

        // For convergence purposes of the Levenberg-Marquardt algorithm, we
        // take common factor M = I + Mg

        // and the gyroscope model can be better expressed as:

        // Ωmeas = M*(Ωtrue + b + G * ftrue)

        // where:
        // bg = M*b --> b = M^-1*bg
        // Gg = M*G --> G = M^-1*Gg

        // Hence
        // Ωmeas - M*b - M*G*ftrue = M*Ωtrue
        // M^-1 * (Ωmeas - M*b - M*G*ftrue) = Ωtrue

        // Notice that M is upper diagonal because Mg is upper diagonal
        // when common axis is assumed


        final JacobianEstimator jacobianEstimator = new JacobianEstimator(new MultiVariateFunctionEvaluatorListener() {
            @Override
            public void evaluate(double[] point, double[] result) throws EvaluationException {
                evaluateCommonAxisWithGDependentCrossBiases(mI, point, result);
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns gravity components
                return BodyKinematics.COMPONENTS;
            }
        });

        final Matrix initialM = Matrix.identity(
                BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        final Matrix invInitialM = Utils.inverse(initialM);
        final Matrix initialBg = getBiasAsMatrix();
        final Matrix initialB = invInitialM.multiplyAndReturnNew(initialBg);
        final Matrix initialGg = getInitialGg();
        final Matrix initialG = invInitialM.multiplyAndReturnNew(initialGg);

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true angular rate + specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns gravity components
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial =
                        new double[COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES];

                // upper diagonal cross coupling errors M
                int k = 0;
                for (int j = 0; j < BodyKinematics.COMPONENTS; j++) {
                    for (int i = 0; i < BodyKinematics.COMPONENTS; i++) {
                        if (i <= j) {
                            initial[k] = initialM.getElementAt(i, j);
                            k++;
                        }
                    }
                }

                // g-dependent cross biases G
                final int num = BodyKinematics.COMPONENTS * BodyKinematics.COMPONENTS;
                for (int i = 0, j = k; i < num; i++, j++) {
                    initial[j] = initialG.getElementAtIndex(i);
                }

                return initial;
            }

            @Override
            public void evaluate(int i, double[] point, double[] result, double[] params, Matrix jacobian)
                    throws EvaluationException {

                mI = i;

                mMeasAngularRateX = point[0];
                mMeasAngularRateY = point[1];
                mMeasAngularRateZ = point[2];

                mFtrueX = point[3];
                mFtrueY = point[4];
                mFtrueZ = point[5];

                jacobianEstimator.jacobian(params, jacobian);

                evaluateCommonAxisWithGDependentCrossBiases(i, params, result);
            }
        });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double m11 = result[0];

        final double m12 = result[1];
        final double m22 = result[2];

        final double m13 = result[3];
        final double m23 = result[4];
        final double m33 = result[5];

        final double g11 = result[6];
        final double g21 = result[7];
        final double g31 = result[8];

        final double g12 = result[9];
        final double g22 = result[10];
        final double g32 = result[11];

        final double g13 = result[12];
        final double g23 = result[13];
        final double g33 = result[14];

        final Matrix m = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        m.setElementAtIndex(0, m11);
        m.setElementAtIndex(1, 0.0);
        m.setElementAtIndex(2, 0.0);

        m.setElementAtIndex(3, m12);
        m.setElementAtIndex(4, m22);
        m.setElementAtIndex(5, 0.0);

        m.setElementAtIndex(6, m13);
        m.setElementAtIndex(7, m23);
        m.setElementAtIndex(8, m33);

        final Matrix g = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        g.setElementAtIndex(0, g11);
        g.setElementAtIndex(1, g21);
        g.setElementAtIndex(2, g31);

        g.setElementAtIndex(3, g12);
        g.setElementAtIndex(4, g22);
        g.setElementAtIndex(5, g32);

        g.setElementAtIndex(6, g13);
        g.setElementAtIndex(7, g23);
        g.setElementAtIndex(8, g33);

        setResult(m, g);
    }

    /**
     * Internal method to perform general calibration when G-dependent cross
     * biases are being estimated.
     *
     * @throws FittingException                              if no convergence to solution is found.
     * @throws InertialNavigatorException                    if inertial navigation fails due to numerical
     *                                                       instabilities.
     * @throws AlgebraException                              if accelerometer parameters prevents fixing
     *                                                       measured accelerometer values due to numerical
     *                                                       instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws com.irurueta.numerical.NotReadyException      if fitter is not ready.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private void calibrateGeneralAndGDependentCrossBiases()
            throws FittingException,
            InertialNavigatorException,
            AlgebraException,
            InvalidSourceAndDestinationFrameTypeException,
            com.irurueta.numerical.NotReadyException,
            InvalidRotationMatrixException {
        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, we
        // take common factor M = I + Mg

        // and the gyroscope model can be better expressed as:

        // Ωmeas = M*(Ωtrue + b + G * ftrue)

        // where:
        // bg = M*b --> b = M^-1*bg
        // Gg = M*G --> G = M^-1*Gg

        // Hence
        // Ωmeas - M*b - M*G*ftrue = M*Ωtrue
        // M^-1 * (Ωmeas - M*b - M*G*ftrue) = Ωtrue

        final JacobianEstimator jacobianEstimator = new JacobianEstimator(new MultiVariateFunctionEvaluatorListener() {
            @Override
            public void evaluate(double[] point, double[] result) throws EvaluationException {
                evaluateGeneralWithGDependentCrossBiases(mI, point, result);
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns gravity components
                return BodyKinematics.COMPONENTS;
            }
        });

        final Matrix initialM = Matrix.identity(
                BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        final Matrix invInitialM = Utils.inverse(initialM);
        final Matrix initialBg = getBiasAsMatrix();
        final Matrix initialB = invInitialM.multiplyAndReturnNew(initialBg);
        final Matrix initialGg = getInitialGg();
        final Matrix initialG = invInitialM.multiplyAndReturnNew(initialGg);

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true angular rate + specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns gravity components
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial =
                        new double[GENERAL_UNKNOWNS_AND_CROSS_BIASES];

                // cross coupling errors M
                final int num = BodyKinematics.COMPONENTS * BodyKinematics.COMPONENTS;
                for (int i = 0; i < num; i++) {
                    initial[i] = initialM.getElementAtIndex(i);
                }

                // g-dependent cross biases G
                for (int i = 0, j = num; i < num; i++, j++) {
                    initial[j] = initialG.getElementAtIndex(i);
                }

                return initial;
            }

            @Override
            public void evaluate(int i, double[] point, double[] result, double[] params, Matrix jacobian)
                    throws EvaluationException {

                mI = i;

                mMeasAngularRateX = point[0];
                mMeasAngularRateY = point[1];
                mMeasAngularRateZ = point[2];

                mFtrueX = point[3];
                mFtrueY = point[4];
                mFtrueZ = point[5];

                jacobianEstimator.jacobian(params, jacobian);

                evaluateGeneralWithGDependentCrossBiases(i, params, result);
            }
        });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double m11 = result[0];
        final double m21 = result[1];
        final double m31 = result[2];

        final double m12 = result[3];
        final double m22 = result[4];
        final double m32 = result[5];

        final double m13 = result[6];
        final double m23 = result[7];
        final double m33 = result[8];

        final double g11 = result[9];
        final double g21 = result[10];
        final double g31 = result[11];

        final double g12 = result[12];
        final double g22 = result[13];
        final double g32 = result[14];

        final double g13 = result[15];
        final double g23 = result[16];
        final double g33 = result[17];

        final Matrix m = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        m.setElementAtIndex(0, m11);
        m.setElementAtIndex(1, m21);
        m.setElementAtIndex(2, m31);

        m.setElementAtIndex(3, m12);
        m.setElementAtIndex(4, m22);
        m.setElementAtIndex(5, m32);

        m.setElementAtIndex(6, m13);
        m.setElementAtIndex(7, m23);
        m.setElementAtIndex(8, m33);

        final Matrix g = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        g.setElementAtIndex(0, g11);
        g.setElementAtIndex(1, g21);
        g.setElementAtIndex(2, g31);

        g.setElementAtIndex(3, g12);
        g.setElementAtIndex(4, g22);
        g.setElementAtIndex(5, g32);

        g.setElementAtIndex(6, g13);
        g.setElementAtIndex(7, g23);
        g.setElementAtIndex(8, g33);

        setResult(m, g);
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed
     * for both the accelerometer and gyroscope and G-dependent cross biases
     * are ignored.
     *
     * @throws FittingException                              if no convergence to solution is found.
     * @throws InertialNavigatorException                    if inertial navigation fails due to numerical
     *                                                       instabilities.
     * @throws AlgebraException                              if accelerometer parameters prevents fixing
     *                                                       measured accelerometer values due to numerical
     *                                                       instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws com.irurueta.numerical.NotReadyException      if fitter is not ready.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private void calibrateCommonAxis()
            throws FittingException,
            InertialNavigatorException,
            AlgebraException,
            InvalidSourceAndDestinationFrameTypeException,
            com.irurueta.numerical.NotReadyException,
            InvalidRotationMatrixException {
        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Where Mg is upper triangular

        // Since G-dependent cross biases are ignored, we can assume that Gg = 0

        // Hence:
        // Ωmeas = bg + (I + Mg) * Ωtrue

        // For convergence purposes of the Levenberg-Marquardt algorithm,
        // the gyroscope model can be better expressed as:
        // Ωmeas = T*K*(Ωtrue + b)
        // Ωmeas = M*(Ωtrue + b)
        // Ωmeas = M*Ωtrue + M*b

        // Hence
        // Ωmeas - M*b = M*Ωtrue

        // M^-1 * (Ωmeas - M*b) = Ωtrue

        // where:
        // M = I + Mg
        // bg = M*b = (I + Mg)*b --> b = M^-1*bg

        // Notice that M is upper diagonal because Mg is upper diagonal
        // when common axis is assumed


        final JacobianEstimator jacobianEstimator = new JacobianEstimator(new MultiVariateFunctionEvaluatorListener() {
            @Override
            public void evaluate(double[] point, double[] result) throws EvaluationException {
                evaluateCommonAxis(mI, point, result);
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns gravity components
                return BodyKinematics.COMPONENTS;
            }
        });

        final Matrix initialM = Matrix.identity(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        final Matrix invInitialM = Utils.inverse(initialM);
        final Matrix initialBg = getBiasAsMatrix();
        final Matrix initialB = invInitialM.multiplyAndReturnNew(initialBg);

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true angular rate + specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns gravity components
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                // upper diagonal cross coupling errors M
                int k = 0;
                for (int j = 0; j < BodyKinematics.COMPONENTS; j++) {
                    for (int i = 0; i < BodyKinematics.COMPONENTS; i++) {
                        if (i <= j) {
                            initial[k] = initialM.getElementAt(i, j);
                            k++;
                        }
                    }
                }

                return initial;
            }

            @Override
            public void evaluate(int i, double[] point, double[] result, double[] params, Matrix jacobian)
                    throws EvaluationException {

                mI = i;

                mMeasAngularRateX = point[0];
                mMeasAngularRateY = point[1];
                mMeasAngularRateZ = point[2];

                mFtrueX = point[3];
                mFtrueY = point[4];
                mFtrueZ = point[5];

                jacobianEstimator.jacobian(params, jacobian);

                evaluateCommonAxis(i, params, result);
            }
        });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double m11 = result[0];

        final double m12 = result[1];
        final double m22 = result[2];

        final double m13 = result[3];
        final double m23 = result[4];
        final double m33 = result[5];

        final Matrix m = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        m.setElementAtIndex(0, m11);
        m.setElementAtIndex(1, 0.0);
        m.setElementAtIndex(2, 0.0);

        m.setElementAtIndex(3, m12);
        m.setElementAtIndex(4, m22);
        m.setElementAtIndex(5, 0.0);

        m.setElementAtIndex(6, m13);
        m.setElementAtIndex(7, m23);
        m.setElementAtIndex(8, m33);

        setResult(m);
    }

    /**
     * Internal method to perform general calibration when G-dependent cross
     * biases are ignored.
     *
     * @throws FittingException                              if no convergence to solution is found.
     * @throws InertialNavigatorException                    if inertial navigation fails due to numerical
     *                                                       instabilities.
     * @throws AlgebraException                              if accelerometer parameters prevents fixing
     *                                                       measured accelerometer values due to numerical
     *                                                       instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws com.irurueta.numerical.NotReadyException      if fitter is not ready.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private void calibrateGeneral()
            throws FittingException,
            InertialNavigatorException,
            AlgebraException,
            InvalidSourceAndDestinationFrameTypeException,
            com.irurueta.numerical.NotReadyException,
            InvalidRotationMatrixException {
        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Since G-dependent cross biases are ignored, we can assume that Gg = 0

        // Hence:
        // Ωmeas = bg + (I + Mg) * Ωtrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // gyroscope model can be better expressed as:
        // Ωmeas = T*K*(Ωtrue + b)
        // Ωmeas = M*(Ωtrue + b)
        // Ωmeas = M*Ωtrue + M*b

        // where:
        // M = I + Mg
        // bg = M*b = (I + Mg)*b --> b = M^-1*bg

        // Hence
        // Ωmeas - M*b = M*Ωtrue

        // M^-1 * (Ωmeas - M*b) = Ωtrue


        final JacobianEstimator jacobianEstimator = new JacobianEstimator(new MultiVariateFunctionEvaluatorListener() {
            @Override
            public void evaluate(double[] point, double[] result) throws EvaluationException {
                evaluateGeneral(mI, point, result);
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns gravity components
                return BodyKinematics.COMPONENTS;
            }
        });

        final Matrix initialM = Matrix.identity(
                BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        final Matrix invInitialM = Utils.inverse(initialM);
        final Matrix initialBg = getBiasAsMatrix();
        final Matrix initialB = invInitialM.multiplyAndReturnNew(initialBg);

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true angular rate + specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns gravity components
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                return initialM.toArray();
            }

            @Override
            public void evaluate(int i, double[] point, double[] result, double[] params, Matrix jacobian)
                    throws EvaluationException {

                mI = i;

                mMeasAngularRateX = point[0];
                mMeasAngularRateY = point[1];
                mMeasAngularRateZ = point[2];

                mFtrueX = point[3];
                mFtrueY = point[4];
                mFtrueZ = point[5];

                jacobianEstimator.jacobian(params, jacobian);

                evaluateGeneral(i, params, result);
            }
        });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double m11 = result[0];
        final double m21 = result[1];
        final double m31 = result[2];

        final double m12 = result[3];
        final double m22 = result[4];
        final double m32 = result[5];

        final double m13 = result[6];
        final double m23 = result[7];
        final double m33 = result[8];

        final Matrix m = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        m.setElementAtIndex(0, m11);
        m.setElementAtIndex(1, m21);
        m.setElementAtIndex(2, m31);

        m.setElementAtIndex(3, m12);
        m.setElementAtIndex(4, m22);
        m.setElementAtIndex(5, m32);

        m.setElementAtIndex(6, m13);
        m.setElementAtIndex(7, m23);
        m.setElementAtIndex(8, m33);

        setResult(m);
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws AlgebraException                              if accelerometer parameters prevents fixing
     *                                                       measured accelerometer values due to numerical
     *                                                       instabilities.
     * @throws InertialNavigatorException                    if inertial navigation fails due to numerical
     *                                                       instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     * @throws InvalidRotationMatrixException                never happens.
     */
    private void setInputData() throws AlgebraException,
            InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        int numMeasurements = 0;
        for (BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence : mSequences) {
            final int count = sequence.getItemsCount();
            if (count > 0 && sequence.getStartBodyAttitude() != null) {
                numMeasurements += (count - 1);
            }
        }

        mNavigationParams = new Matrix(numMeasurements, 10);

        final double[] measuredF = new double[BodyKinematics.COMPONENTS];
        final double[] measuredAngularRate = new double[
                BodyKinematics.COMPONENTS];
        final double[] fixedF = new double[BodyKinematics.COMPONENTS];
        final double[] fixedAngularRate = new double[BodyKinematics.COMPONENTS];
        final Matrix ba = getAccelerometerBiasAsMatrix();
        final Matrix ma = getAccelerometerMa();
        final Matrix bg = getBiasAsMatrix();
        final Matrix mg = getInitialMg();
        final Matrix gg = getInitialGg();
        final Matrix x = new Matrix(numMeasurements,
                2 * BodyKinematics.COMPONENTS);
        final Matrix y = new Matrix(numMeasurements,
                BodyKinematics.COMPONENTS);
        final double[] standardDeviations = new double[numMeasurements];

        final ECEFFrame frame = new ECEFFrame();
        final double posX = mPosition.getX();
        final double posY = mPosition.getY();
        final double posZ = mPosition.getZ();

        CoordinateTransformation oldC;
        double oldPosX;
        double oldPosY;
        double oldPosZ;
        double oldVx;
        double oldVy;
        double oldVz;

        int i = 0;
        for (final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence : mSequences) {
            if (sequence.getStartBodyAttitude() == null) {
                continue;
            }

            final List<StandardDeviationTimedBodyKinematics> sortedMeasurements =
                    sequence.getSortedItems();
            if (sortedMeasurements.isEmpty()) {
                continue;
            }

            // initialize position and attitude at the start of the sequence
            oldPosX = posX;
            oldPosY = posY;
            oldPosZ = posZ;
            oldVx = 0.0;
            oldVy = 0.0;
            oldVz = 0.0;
            // start body attitude below is a body to ECEF transformation
            oldC = sequence.getStartBodyAttitude();

            boolean first = true;
            double previousTimestamp = 0.0;
            double timestamp;
            for (final StandardDeviationTimedBodyKinematics measurement : sortedMeasurements) {
                if (first) {
                    previousTimestamp = measurement.getTimestampSeconds();
                    first = false;
                    continue;
                }

                timestamp = measurement.getTimestampSeconds();
                final double timeInterval = timestamp - previousTimestamp;
                previousTimestamp = timestamp;

                final BodyKinematics kinematics = measurement.getKinematics();
                measuredF[0] = kinematics.getFx();
                measuredF[1] = kinematics.getFy();
                measuredF[2] = kinematics.getFz();
                mAccelerationFixer.fix(measuredF, ba, ma, fixedF);

                final double fixedFx = fixedF[0];
                final double fixedFy = fixedF[1];
                final double fixedFz = fixedF[2];

                final double measWx = kinematics.getAngularRateX();
                final double measWy = kinematics.getAngularRateY();
                final double measWz = kinematics.getAngularRateZ();
                measuredAngularRate[0] = measWx;
                measuredAngularRate[1] = measWy;
                measuredAngularRate[2] = measWz;

                mAngularRateFixer.fix(measuredAngularRate, fixedF,
                        bg, mg, gg, fixedAngularRate);

                final double fixedWx = fixedAngularRate[0];
                final double fixedWy = fixedAngularRate[1];
                final double fixedWz = fixedAngularRate[2];

                // set IMU (fixed accelerometer values + gyroscope values)
                // into input matrix
                x.setElementAt(i, 0, measWx);
                x.setElementAt(i, 1, measWy);
                x.setElementAt(i, 2, measWz);

                x.setElementAt(i, 3, fixedFx);
                x.setElementAt(i, 4, fixedFy);
                x.setElementAt(i, 5, fixedFz);

                final double oldRoll = oldC.getRollEulerAngle();
                final double oldPitch = oldC.getPitchEulerAngle();
                final double oldYaw = oldC.getYawEulerAngle();
                mNavigationParams.setElementAt(i, 0, timeInterval);
                mNavigationParams.setElementAt(i, 1, oldPosX);
                mNavigationParams.setElementAt(i, 2, oldPosY);
                mNavigationParams.setElementAt(i, 3, oldPosZ);
                mNavigationParams.setElementAt(i, 4, oldRoll);
                mNavigationParams.setElementAt(i, 5, oldPitch);
                mNavigationParams.setElementAt(i, 6, oldYaw);
                mNavigationParams.setElementAt(i, 7, oldVx);
                mNavigationParams.setElementAt(i, 8, oldVy);
                mNavigationParams.setElementAt(i, 9, oldVz);

                // estimate new position and attitude and store it into
                // an ECEF frame
                ECEFInertialNavigator.navigateECEF(timeInterval,
                        oldPosX, oldPosY, oldPosZ, oldC,
                        oldVx, oldVy, oldVz,
                        fixedFx, fixedFy, fixedFz,
                        fixedWx, fixedWy, fixedWz, frame);

                // estimate gravity assuming static device (zero velocity
                // and position and attitude does not change)
                estimateGravity(timeInterval, frame);

                // store quaternion into output matrix
                y.setElementAt(i, 0, mGravity[0]);
                y.setElementAt(i, 1, mGravity[1]);
                y.setElementAt(i, 2, mGravity[2]);

                standardDeviations[i] =
                        measurement.getAngularRateStandardDeviation();

                i++;

                oldPosX = frame.getX();
                oldPosY = frame.getY();
                oldPosZ = frame.getZ();
                oldVx = frame.getVx();
                oldVy = frame.getVy();
                oldVz = frame.getVz();
                oldC = frame.getCoordinateTransformation();
            }
        }

        mFitter.setInputData(x, y, standardDeviations);
    }

    /**
     * Estimates gravity components for provided ECEF frame during
     * a given time interval.
     *
     * @param timeInterval a time interval expressed in seconds.
     * @param frame        an ECEF frame.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     */
    private void estimateGravity(
            final double timeInterval, final ECEFFrame frame)
            throws InvalidSourceAndDestinationFrameTypeException {

        frame.getECEFPosition(mGravityPosition);
        frame.getCoordinateTransformation(mGravityC);

        mGravityFrame.setPosition(mGravityPosition);
        mGravityFrame.setCoordinateTransformation(mGravityC);

        ECEFKinematicsEstimator.estimateKinematics(timeInterval,
                mGravityFrame, mGravityFrame, mGravityKinematics);

        mGravity[0] = mGravityKinematics.getFx();
        mGravity[1] = mGravityKinematics.getFy();
        mGravity[2] = mGravityKinematics.getFz();
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

    /**
     * Makes proper conversion of internal cross-coupling, bias and g-dependent
     * cross bias matrices.
     *
     * @param m internal scaling and cross-coupling matrix.
     * @param g internal g-dependent cross bias matrix.
     * @throws AlgebraException if a numerical instability occurs.
     */
    private void setResult(final Matrix m, final Matrix g)
            throws AlgebraException {
        setResult(m);

        // Gg = M*G
        m.multiply(g, mEstimatedGg);
    }

    /**
     * Makes proper conversion of internal cross-coupling and bias matrices.
     *
     * @param m internal scaling and cross-coupling matrix.
     * @throws AlgebraException if a numerical instability occurs.
     */
    private void setResult(final Matrix m) throws AlgebraException {
        // Because:
        // M = I + Mg
        // b = M^-1*bg

        // Then:
        // Mg = M - I
        // bg = M*b

        if (mEstimatedMg == null) {
            mEstimatedMg = m;
        } else {
            mEstimatedMg.copyFrom(m);
        }

        for (int i = 0; i < BodyKinematics.COMPONENTS; i++) {
            mEstimatedMg.setElementAt(i, i,
                    mEstimatedMg.getElementAt(i, i) - 1.0);
        }

        if (mEstimatedGg == null) {
            mEstimatedGg = new Matrix(
                    BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } else {
            mEstimatedGg.initialize(0.0);
        }

        mEstimatedCovariance = mFitter.getCovar();
        mEstimatedChiSq = mFitter.getChisq();
    }

    /**
     * Computes estimated attitude rotation parameters for the
     * general case when G-dependent cross biases are taken into account.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i      row position.
     * @param params array containing parameters for the general purpose case
     *               when G-dependent cross biases are taken into account. Must
     *               have length 21.
     * @param result array containing estimated rotation parameters of a
     *               normalized quaternion. Must have length 4.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private void evaluateGeneralWithGDependentCrossBiases(
            final int i, final double[] params, final double[] result)
            throws EvaluationException {

        final double m11 = params[0];
        final double m21 = params[1];
        final double m31 = params[2];

        final double m12 = params[3];
        final double m22 = params[4];
        final double m32 = params[5];

        final double m13 = params[6];
        final double m23 = params[7];
        final double m33 = params[8];

        final double g11 = params[9];
        final double g21 = params[10];
        final double g31 = params[11];

        final double g12 = params[12];
        final double g22 = params[13];
        final double g32 = params[14];

        final double g13 = params[15];
        final double g23 = params[16];
        final double g33 = params[17];

        evaluate(i, m11, m21, m31, m12, m22, m32,
                m13, m23, m33, g11, g21, g31, g12, g22, g32,
                g13, g23, g33, result);
    }

    /**
     * Computes estimated attitude rotation parameters when
     * common z-axis is assumed and G-dependent cross biases are taken into
     * account.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i      row position.
     * @param params array containing parameters for the general purpose case
     *               when G-dependent cross biases are taken into account. Must
     *               have length 18.
     * @param result array containing estimated rotation parameters of a
     *               normalized quaternion. Must have length 4.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private void evaluateCommonAxisWithGDependentCrossBiases(
            final int i, final double[] params, final double[] result)
            throws EvaluationException {

        final double m11 = params[0];

        final double m12 = params[1];
        final double m22 = params[2];

        final double m13 = params[3];
        final double m23 = params[4];
        final double m33 = params[5];

        final double g11 = params[6];
        final double g21 = params[7];
        final double g31 = params[8];

        final double g12 = params[9];
        final double g22 = params[10];
        final double g32 = params[11];

        final double g13 = params[12];
        final double g23 = params[13];
        final double g33 = params[14];

        evaluate(i, m11, 0.0, 0.0, m12, m22, 0.0,
                m13, m23, m33, g11, g21, g31, g12, g22, g32,
                g13, g23, g33, result);
    }

    /**
     * Computes estimated attitude rotation parameters for the general
     * case when G-dependent cross biases are ignored.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i      row position.
     * @param params array containing current parameters for the general purpose case
     *               when G-dependent cross biases are ignored. Must have length 12.
     * @param result array containing estimated rotation parameters of a
     *               normalized quaternion. Must have length 4.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private void evaluateGeneral(
            final int i, final double[] params, final double[] result)
            throws EvaluationException {

        final double m11 = params[0];
        final double m21 = params[1];
        final double m31 = params[2];

        final double m12 = params[3];
        final double m22 = params[4];
        final double m32 = params[5];

        final double m13 = params[6];
        final double m23 = params[7];
        final double m33 = params[8];

        evaluate(i, m11, m21, m31, m12, m22, m32,
                m13, m23, m33, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, result);
    }

    /**
     * Computes estimated attitude rotation parameters when
     * common z-axis is assumed and G-dependent cross biases are ignored.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i      row position.
     * @param params array containing current parameters for the common z-axis case
     *               when G-dependent cross biases are ignored. Must have length 9.
     * @param result array containing estimated rotation parameters of a
     *               normalized quaternion. Must have length 4.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private void evaluateCommonAxis(
            final int i, final double[] params, final double[] result)
            throws EvaluationException {
        final double m11 = params[0];

        final double m12 = params[1];
        final double m22 = params[2];

        final double m13 = params[3];
        final double m23 = params[4];
        final double m33 = params[5];

        evaluate(i, m11, 0.0, 0.0, m12, m22, 0.0,
                m13, m23, m33, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, result);
    }

    /**
     * Computes estimated attitude rotation parameters using current
     * measured specific force and angular rate and provided parameters.
     * Since we are calibrating, we can obtain fixed angular rates using
     * provided parameters and a {@link LevelingEstimator2} to obtain
     * a body attitude (that must be converted from
     * Local Navigation -> Body frame transformation to a Body frame -> ECEF
     * by taking the inverse of estimated attitude of the leveling estimator
     * and converting it to ECEF frame using current position and zero
     * velocity).
     *
     * @param i      row position.
     * @param m11    element 1,1 of cross-coupling error matrix.
     * @param m21    element 2,1 of cross-coupling error matrix.
     * @param m31    element 3,1 of cross-coupling error matrix.
     * @param m12    element 1,2 of cross-coupling error matrix.
     * @param m22    element 2,2 of cross-coupling error matrix.
     * @param m32    element 3,2 of cross-coupling error matrix.
     * @param m13    element 1,3 of cross-coupling error matrix.
     * @param m23    element 2,3 of cross-coupling error matrix.
     * @param m33    element 3,3 of cross-coupling error matrix.
     * @param g11    element 1,1 of g-dependent cross bias matrix.
     * @param g21    element 2,1 of g-dependent cross bias matrix.
     * @param g31    element 3,1 of g-dependent cross bias matrix.
     * @param g12    element 1,2 of g-dependent cross bias matrix.
     * @param g22    element 2,2 of g-dependent cross bias matrix.
     * @param g32    element 3,2 of g-dependent cross bias matrix.
     * @param g13    element 1,3 of g-dependent cross bias matrix.
     * @param g23    element 2,3 of g-dependent cross bias matrix.
     * @param g33    element 3,3 of g-dependent cross bias matrix.
     * @param result array containing estimated rotation parameters of a
     *               normalized quaternion. Must have length 4.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private void evaluate(final int i,
                          final double m11, final double m21, final double m31,
                          final double m12, final double m22, final double m32,
                          final double m13, final double m23, final double m33,
                          final double g11, final double g21, final double g31,
                          final double g12, final double g22, final double g32,
                          final double g13, final double g23, final double g33,
                          final double[] result)
            throws EvaluationException {

        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue
        // Ωmeas = M*(Ωtrue + b + G * ftrue)

        // M = I + Mg
        // bg = M*b --> b = M^-1*bg
        // Gg = M*G --> G = M^-1*Gg

        // Ωtrue = M^-1 * Ωmeas - b - G*ftrue

        try {
            if (mMeasAngularRate == null) {
                mMeasAngularRate = new Matrix(
                        BodyKinematics.COMPONENTS, 1);
            }
            if (mM == null) {
                mM = new Matrix(BodyKinematics.COMPONENTS,
                        BodyKinematics.COMPONENTS);
            }
            if (mInvM == null) {
                mInvM = new Matrix(BodyKinematics.COMPONENTS,
                        BodyKinematics.COMPONENTS);
            }
            if (mB == null) {
                mB = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (mG == null) {
                mG = new Matrix(BodyKinematics.COMPONENTS,
                        BodyKinematics.COMPONENTS);
            }
            if (mTrueAngularRate == null) {
                mTrueAngularRate = new Matrix(
                        BodyKinematics.COMPONENTS, 1);
            }
            if (mFtrue == null) {
                mFtrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (mTmp == null) {
                mTmp = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            mMeasAngularRate.setElementAtIndex(0, mMeasAngularRateX);
            mMeasAngularRate.setElementAtIndex(1, mMeasAngularRateY);
            mMeasAngularRate.setElementAtIndex(2, mMeasAngularRateZ);

            mFtrue.setElementAtIndex(0, mFtrueX);
            mFtrue.setElementAtIndex(1, mFtrueY);
            mFtrue.setElementAtIndex(2, mFtrueZ);

            mM.setElementAt(0, 0, m11);
            mM.setElementAt(1, 0, m21);
            mM.setElementAt(2, 0, m31);

            mM.setElementAt(0, 1, m12);
            mM.setElementAt(1, 1, m22);
            mM.setElementAt(2, 1, m32);

            mM.setElementAt(0, 2, m13);
            mM.setElementAt(1, 2, m23);
            mM.setElementAt(2, 2, m33);

            Utils.inverse(mM, mInvM);

            mB.setElementAtIndex(0, mBiasX);
            mB.setElementAtIndex(1, mBiasY);
            mB.setElementAtIndex(2, mBiasZ);

            mG.setElementAt(0, 0, g11);
            mG.setElementAt(1, 0, g21);
            mG.setElementAt(2, 0, g31);

            mG.setElementAt(0, 1, g12);
            mG.setElementAt(1, 1, g22);
            mG.setElementAt(2, 1, g32);

            mG.setElementAt(0, 2, g13);
            mG.setElementAt(1, 2, g23);
            mG.setElementAt(2, 2, g33);

            mG.multiply(mFtrue, mTmp);
            mInvM.multiply(mMeasAngularRate, mTrueAngularRate);
            mTrueAngularRate.subtract(mB);
            mTrueAngularRate.subtract(mTmp);

            final double trueAngularRateX = mTrueAngularRate
                    .getElementAtIndex(0);
            final double trueAngularRateY = mTrueAngularRate
                    .getElementAtIndex(1);
            final double trueAngularRateZ = mTrueAngularRate
                    .getElementAtIndex(2);

            final CoordinateTransformation cnb = LevelingEstimator
                    .getAttitude(mFtrueX, mFtrueY, mFtrueZ,
                            trueAngularRateX, trueAngularRateY, trueAngularRateZ);
            final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

            final NEDPosition nedPosition = getNedPosition();
            final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
            if (mEcefFrame == null) {
                mEcefFrame = new ECEFFrame();
            }
            NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mEcefFrame);

            final double timeInterval = mNavigationParams.getElementAt(i, 0);
            /*final double oldPosX = mNavigationParams.getElementAt(i, 1);
            final double oldPosY = mNavigationParams.getElementAt(i, 2);
            final double oldPosZ = mNavigationParams.getElementAt(i, 3);
            final double oldRoll = mNavigationParams.getElementAt(i, 4);
            final double oldPitch = mNavigationParams.getElementAt(i, 5);
            final double oldYaw = mNavigationParams.getElementAt(i, 6);
            final double oldVx = mNavigationParams.getElementAt(i, 7);
            final double oldVy = mNavigationParams.getElementAt(i, 8);
            final double oldVz = mNavigationParams.getElementAt(i, 9);

            if (mEcefFrame == null) {
                mEcefFrame = new ECEFFrame();
                mOldC = mEcefFrame.getCoordinateTransformation();
            }

            mOldC.setEulerAngles(oldRoll, oldPitch, oldYaw);

            ECEFInertialNavigator.navigateECEF(timeInterval,
                    oldPosX, oldPosY, oldPosZ, mOldC,
                    oldVx, oldVy, oldVz,
                    mFtrueX, mFtrueY, mFtrueZ,
                    trueAngularRateX, trueAngularRateY, trueAngularRateZ,
                    mEcefFrame);*/

            estimateGravity(timeInterval, mEcefFrame);

            System.arraycopy(mGravity, 0, result, 0,
                    mGravity.length);

        } catch (final AlgebraException
                | InvalidSourceAndDestinationFrameTypeException e) {
//                | InertialNavigatorException e) {
            throw new EvaluationException(e);
        }
    }
}
