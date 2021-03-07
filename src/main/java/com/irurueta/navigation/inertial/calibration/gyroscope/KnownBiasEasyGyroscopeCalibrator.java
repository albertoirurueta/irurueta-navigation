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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationFixer;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Estimates gyroscope cross couplings, scaling factors and
 * G-dependent cross biases introduced on the gyroscope by the specific
 * forces sensed by the accelerometer.
 * <p>
 * This calibrator assumes that the IMU is at a more or less fixed location on
 * Earth, and evaluates sequences of measured body kinematics to perform
 * calibration for unknown orientations on those provided sequences.
 * Each provided sequence will be preceded by a static period where mean
 * specific force will be measured to determine gravity (and hence partial
 * body attitude).
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
public class KnownBiasEasyGyroscopeCalibrator implements GyroscopeNonLinearCalibrator,
        KnownBiasGyroscopeCalibrator, OrderedBodyKinematicsSequenceGyroscopeCalibrator,
        AccelerometerDependentGyroscopeCalibrator {

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
    private final LevenbergMarquardtMultiDimensionFitter mFitter =
            new LevenbergMarquardtMultiDimensionFitter();

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
     * X-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
     */
    private double mBiasX;

    /**
     * Y-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
     */
    private double mBiasY;

    /**
     * Z-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
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
    private List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mSequences;

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
    private KnownBiasEasyGyroscopeCalibratorListener mListener;

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
     * Estimated mean square error respect to provided measurements.
     */
    private double mEstimatedMse;

    /**
     * Indicates whether calibrator is running.
     */
    private boolean mRunning;

    /**
     * Acceleration fixer.
     */
    private final AccelerationFixer mAccelerationFixer =
            new AccelerationFixer();

    /**
     * Index of current point being evaluated.
     */
    private int mI;

    /**
     * Holds integrated rotation of a sequence.
     */
    private final Quaternion mQ = new Quaternion();

    /**
     * Contains a copy of input sequences where fixed body kinematics will be updated.
     */
    private List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mFixedSequences;

    /**
     * Contains measured specific force on a sample within a sequence
     * expressed as a 3x1 matrix.
     */
    private Matrix mMeasuredSpecificForce;

    /**
     * Contains fixed specific force for a sample within a sequence
     * using provided accelerometer parameters and expressed as a 3x1
     * matrix.
     */
    private Matrix mTrueSpecificForce;

    /**
     * Contains measured angular rate on a sample within a sequence
     * expressed as a 3x1 matrix.
     */
    private Matrix mMeasuredAngularRate;

    /**
     * Contains fixed specific force for a samplewithin a sequence
     * using current parameters being estimated.
     */
    private Matrix mTrueAngularRate;

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
     * Internally holds angular rate bias due to g-dependent cross biases
     */
    private Matrix mTmp;

    /**
     * Holds normalized gravity versor at the start of a sequence.
     * This is used to compute rotations of gravity versor during
     * calibration.
     */
    private final InhomogeneousPoint3D mStartPoint = new InhomogeneousPoint3D();

    /**
     * Holds normalized gravity versor at the end of a sequence.
     * This is used to compute rotations of gravity versor during
     * calibration.
     */
    private final InhomogeneousPoint3D mEndPoint = new InhomogeneousPoint3D();

    /**
     * Contains expected gravity versor obtained from measurements fixed
     * using known accelerometer parameters.
     * This is reused for memory efficiency purposes during calibration.
     */
    private final InhomogeneousPoint3D mExpectedEndPoint = new InhomogeneousPoint3D();

    /**
     * Array containing normalized gravity versor before (former 3 values)
     * and after (latter 3 values) a given sequence.
     * This is used during calibration.
     */
    private double[] mPoint;

    /**
     * Constructor.
     */
    public KnownBiasEasyGyroscopeCalibrator() {
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this();
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, bias, initialMg, initialGg);
        mListener = listener;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this();
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, bias, initialMg, initialGg);
        mListener = listener;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(sequences, bias, initialMg, initialGg);
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        mListener = listener;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(sequences, bias, initialMg, initialGg);
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        mListener = listener;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(sequences, bias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
        mListener = listener;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(sequences, bias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
        mListener = listener;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
        mListener = listener;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(sequences, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
    public KnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final KnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
        mListener = listener;
    }

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known x-coordinate of accelerometer bias.
     */
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
     * Gets x-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return x-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasX() {
        return mBiasX;
    }

    /**
     * Sets x-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasX x-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final double biasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return y-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasY() {
        return mBiasY;
    }

    /**
     * Sets y-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasY y-coordinate of gyroscope known  bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final double biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = biasY;
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return z-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasZ() {
        return mBiasZ;
    }

    /**
     * Sets z-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasZ z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = biasZ;
    }

    /**
     * Gets x-coordinate of gyroscope known bias.
     *
     * @return x-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedX() {
        return new AngularSpeed(mBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets x-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(mBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets x-coordinate of gyroscope known bias.
     *
     * @param biasX x-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final AngularSpeed biasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = convertAngularSpeed(biasX);
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     *
     * @return y-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedY() {
        return new AngularSpeed(mBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(mBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets y-coordinate of gyroscope known bias.
     *
     * @param biasY y-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final AngularSpeed biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = convertAngularSpeed(biasY);
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     *
     * @return initial z-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedZ() {
        return new AngularSpeed(mBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets z-coordinate of gyroscope known bias.
     *
     * @param biasZ z-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final AngularSpeed biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Sets known bias coordinates of gyroscope expressed in
     * radians per second (rad/s).
     *
     * @param biasX x-coordinate of gyroscope known bias.
     * @param biasY y-coordinate of gyroscope known bias.
     * @param biasZ z-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(
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
     * @param biasX x-coordinate of gyroscope known bias.
     * @param biasY y-coordinate of gyroscope known bias.
     * @param biasZ z-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(
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
     * Gets known gyroscope bias.
     *
     * @return known gyroscope bias.
     */
    public AngularSpeedTriad getBiasAsTriad() {
        return new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND,
                mBiasX, mBiasY, mBiasZ);
    }

    /**
     * Gets known gyroscope bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsTriad(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(mBiasX, mBiasY, mBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known gyroscope bias.
     *
     * @param bias gyroscope bias to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(final AngularSpeedTriad bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasX = convertAngularSpeed(bias.getValueX(), bias.getUnit());
        mBiasY = convertAngularSpeed(bias.getValueY(), bias.getUnit());
        mBiasZ = convertAngularSpeed(bias.getValueZ(), bias.getUnit());
    }

    /**
     * Gets initial x scaling factor of gyroscope.
     *
     * @return initial x scaling factor of gyroscope.
     */
    @Override
    public double getInitialSx() {
        return mInitialSx;
    }

    /**
     * Sets initial x scaling factor of gyroscope.
     *
     * @param initialSx initial x scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
    public double getInitialSy() {
        return mInitialSy;
    }

    /**
     * Sets initial y scaling factor of gyroscope.
     *
     * @param initialSy initial y scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
    public double getInitialSz() {
        return mInitialSz;
    }

    /**
     * Sets initial z scaling factor of gyroscope.
     *
     * @param initialSz initial z scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
    public double getInitialMxy() {
        return mInitialMxy;
    }

    /**
     * Sets initial x-y cross coupling error of gyroscope.
     *
     * @param initialMxy initial x-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
    public double getInitialMxz() {
        return mInitialMxz;
    }

    /**
     * Sets initial x-z cross coupling error of gyroscope.
     *
     * @param initialMxz initial x-z cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
    public double getInitialMyx() {
        return mInitialMyx;
    }

    /**
     * Sets initial y-x cross coupling error of gyroscope.
     *
     * @param initialMyx initial y-x cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
    public double getInitialMyz() {
        return mInitialMyz;
    }

    /**
     * Sets initial y-z cross coupling error of gyroscope.
     *
     * @param initialMyz initial y-z cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
    public double getInitialMzx() {
        return mInitialMzx;
    }

    /**
     * Sets initial z-x cross coupling error of gyroscope.
     *
     * @param initialMzx initial z-x cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
    public double getInitialMzy() {
        return mInitialMzy;
    }

    /**
     * Sets initial z-y cross coupling error of gyroscope.
     *
     * @param initialMzy initial z-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
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
    @Override
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
    @Override
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
     * Gets gyroscope known bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of gyroscope known bias.
     */
    @Override
    public double[] getBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets gyroscope known bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void getBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = mBiasX;
        result[1] = mBiasY;
        result[2] = mBiasZ;
    }

    /**
     * Sets gyroscope known bias to be used to find a solution as
     * an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
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
     * Gets gyroscope known bias as a column matrix.
     * Matrix values are expressed in radians per second (rad/s).
     *
     * @return initial gyroscope bias to be used to find a solution as a
     * column matrix.
     */
    @Override
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
     * Gets gyroscope known bias as a column matrix.
     * Matrix values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
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
     * Sets gyroscope known bias as a column matrix.
     * Matrix values are expressed in radians per second (rad/s).
     *
     * @param bias gyroscope known bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
    public List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> getSequences() {
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
    @Override
    public void setSequences(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mSequences = sequences;
    }

    /**
     * Indicates the type of measurement or sequence used by this calibrator.
     *
     * @return type of measurement or sequence used by this calibrator.
     */
    @Override
    public GyroscopeCalibratorMeasurementOrSequenceType getMeasurementOrSequenceType() {
        return GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE;
    }

    /**
     * Indicates whether this calibrator requires ordered measurements or sequences
     * in a list or not.
     *
     * @return true if measurements or sequences must be ordered, false otherwise.
     */
    @Override
    public boolean isOrderedMeasurementsOrSequencesRequired() {
        return true;
    }

    /**
     * Indicates whether this calibrator requires quality scores for each
     * measurement/sequence or not.
     *
     * @return true if quality scores are required, false otherwise.
     */
    @Override
    public boolean isQualityScoresRequired() {
        return false;
    }

    /**
     * Indicates whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @return true if z-axis is assumed to be common for accelerometer and gyroscope,
     * false otherwise.
     */
    @Override
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
    @Override
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
    public KnownBiasEasyGyroscopeCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final KnownBiasEasyGyroscopeCalibratorListener listener)
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
    @Override
    public int getMinimumRequiredMeasurementsOrSequences() {
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
    @Override
    public boolean isReady() {
        return mSequences != null
                && mSequences.size() >= getMinimumRequiredMeasurementsOrSequences();
    }

    /**
     * Indicates whether calibrator is currently running or not.
     *
     * @return true if calibrator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return mRunning;
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

        try {
            mRunning = true;

            reset();

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
                | AlgebraException
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
    @Override
    public Matrix getEstimatedMg() {
        return mEstimatedMg;
    }

    /**
     * Gets estimated gyroscope x-axis scale factor.
     *
     * @return estimated gyroscope x-axis scale factor or null
     * if not available.
     */
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
    public Matrix getEstimatedGg() {
        return mEstimatedGg;
    }

    /**
     * Gets estimated covariance matrix for estimated parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32, gg13, gg23, gg33.
     *
     * @return estimated covariance matrix for estimated parameters.
     */
    @Override
    public Matrix getEstimatedCovariance() {
        return mEstimatedCovariance;
    }

    /**
     * Gets estimated chi square value.
     *
     * @return estimated chi square value.
     */
    @Override
    public double getEstimatedChiSq() {
        return mEstimatedChiSq;
    }

    /**
     * Gets estimated mean square error respect to provided measurements.
     *
     * @return estimated mean square error respect to provided measurements.
     */
    @Override
    public double getEstimatedMse() {
        return mEstimatedMse;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed
     * for both the accelerometer and gyroscope and when G-dependent cross
     * biases are being estimated.
     *
     * @throws AlgebraException                         if accelerometer parameters prevent fixing
     *                                                  measured accelerometer values due to numerical instabilities.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateCommonAxisAndGDependentCrossBiases()
            throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
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

        final GradientEstimator gradientEstimator = new GradientEstimator(
                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(final double[] params)
                            throws EvaluationException {
                        return evaluateCommonAxisWithGDependentCrossBiases(mI, params);
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
        final Matrix initialGg = getInitialGg();
        final Matrix initialG = invInitialM.multiplyAndReturnNew(initialGg);

        final Matrix bg = getBiasAsMatrix();
        mB = invInitialM.multiplyAndReturnNew(bg);

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Before and after normalized gravity versors
                        return 2 * BodyKinematics.COMPONENTS;
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
                    public double evaluate(
                            final int i, final double[] point,
                            final double[] params, final double[] derivatives)
                            throws EvaluationException {
                        mI = i;

                        // point contains fixed gravity versor values for current
                        // sequence
                        mPoint = point;

                        gradientEstimator.gradient(params, derivatives);

                        return evaluateCommonAxisWithGDependentCrossBiases(i, params);
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

        // at this point covariance is expressed in terms of M and G, and must
        // be expressed in terms of Mg and Gg.
        // We know that:
        //Mg = M - I
        //Gg = M * G

        //M = [m11  m12  m13]
        //    [0    m22  m23]
        //    [0    0    m33]

        //G = [g11  g12  g13]
        //    [g21  g22  g23]
        //    [g31  g32  g33]

        //Mg = [sx  mxy  mxz] = [m11 - 1    m12         m13     ]
        //     [myx	sy	 myz]   [0          m22 - 1     m23     ]
        //     [mzx	mzy  sz ]   [0          0           m33 - 1 ]

        //Gg = [gg11  gg12  gg13] = [m11  m12  m13][g11  g12  g13]
        //     [gg21  gg22  gg23]   [0    m22  m23][g21  g22  g23]
        //     [gg31  gg32  gg33]   [0    0    m33][g31  g32  g33]

        // Defining the linear application:
        // F(M, G) = F(m11, m12, m22, m32=0, m13, m23, m33, g11, g21, g31, g12, g22, g32, g13, g23, g33)
        // as:
        // [sx] =   [m11 - 1]
        // [sy]     [m22 - 1]
        // [sz]     [m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [0]
        // [myz]    [m23]
        // [mzx]    [0]
        // [mzy]    [0]
        // [gg11]   [m11 * g11 + m12 * g21 + m13 * g31]
        // [gg21]   [            m22 * g21 + m23 * g31]
        // [gg31]   [                        m33 * g31]
        // [gg12]   [m11 * g12 + m12 * g22 + m13 * g32]
        // [gg22]   [            m22 * g22 + n23 * g32]
        // [gg32]   [                        m33 * g32]
        // [gg13]   [m11 * g13 + m12 * g23 + m13 * g33]
        // [gg23]   [            m22 * g23 + m23 * g33]
        // [gg33]   [                        m33 * g33]

        // Then the Jacobian of F(M, G) is:
        //J = [1    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    1    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    0    1    0    0    0    0    0    0    0    0    0  ]
        //    [0    1    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    1    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    1    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [g11  g21  0    g31  0    0    m11  m12  m13  0    0    0    0    0    0  ]
        //    [0    0    g21  0    g31  0    0    m22  m23  0    0    0    0    0    0  ]
        //    [0    0    0    0    0    g31  0    0    m33  0    0    0    0    0    0  ]
        //    [g12  g22  0    g32  0    0    0    0    0    m11  m12  m13  0    0    0  ]
        //    [0    0    g22  0    g32  0    0    0    0    0    m22  m23  0    0    0  ]
        //    [0    0    0    0    0    g32  0    0    0    0    0    m33  0    0    0  ]
        //    [g13  g23  0    g33  0    0    0    0    0    0    0    0    m11  m12  m13]
        //    [0    0    g23  0    g33  0    0    0    0    0    0    0    0    m22  m23]
        //    [0    0    0    0    0    g33  0    0    0    0    0    0    0    0    m33]

        // We know that the propagated covariance is J * Cov * J', hence:
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS_AND_CROSS_BIASES, COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES);

        jacobian.setElementAt(0, 0, 1.0);
        jacobian.setElementAt(1, 2, 1.0);
        jacobian.setElementAt(2, 5, 1.0);

        jacobian.setElementAt(3, 1, 1.0);
        jacobian.setElementAt(4, 3, 1.0);

        jacobian.setElementAt(6, 4, 1.0);

        jacobian.setElementAt(9, 0, g11);
        jacobian.setElementAt(9, 1, g21);
        jacobian.setElementAt(9, 3, g31);
        jacobian.setElementAt(9, 6, m11);
        jacobian.setElementAt(9, 7, m12);
        jacobian.setElementAt(9, 8, m13);

        jacobian.setElementAt(10, 2, g21);
        jacobian.setElementAt(10, 4, g31);
        jacobian.setElementAt(10, 7, m22);
        jacobian.setElementAt(10, 8, m23);

        jacobian.setElementAt(11, 5, g31);
        jacobian.setElementAt(11, 8, m33);

        jacobian.setElementAt(12, 0, g12);
        jacobian.setElementAt(12, 1, g22);
        jacobian.setElementAt(12, 3, g32);
        jacobian.setElementAt(12, 9, m11);
        jacobian.setElementAt(12, 10, m12);
        jacobian.setElementAt(12, 11, m13);

        jacobian.setElementAt(13, 2, g22);
        jacobian.setElementAt(13, 4, g32);
        jacobian.setElementAt(13, 10, m22);
        jacobian.setElementAt(13, 11, m23);

        jacobian.setElementAt(14, 5, g32);
        jacobian.setElementAt(14, 11, m33);

        jacobian.setElementAt(15, 0, g13);
        jacobian.setElementAt(15, 1, g23);
        jacobian.setElementAt(15, 3, g33);
        jacobian.setElementAt(15, 12, m11);
        jacobian.setElementAt(15, 13, m12);
        jacobian.setElementAt(15, 14, m13);

        jacobian.setElementAt(16, 2, g23);
        jacobian.setElementAt(16, 4, g33);
        jacobian.setElementAt(16, 13, m22);
        jacobian.setElementAt(16, 14, m23);

        jacobian.setElementAt(17, 5, g33);
        jacobian.setElementAt(17, 14, m33);

        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
    }

    /**
     * Internal method to perform general calibration when G-dependent cross
     * biases are being estimated.
     *
     * @throws AlgebraException                         if accelerometer parameters prevent fixing
     *                                                  measured accelerometer values due to numerical instabilities.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateGeneralAndGDependentCrossBiases()
            throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
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

        final GradientEstimator gradientEstimator = new GradientEstimator(
                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] params)
                            throws EvaluationException {
                        return evaluateGeneralWithGDependentCrossBiases(mI, params);
                    }
                });

        final Matrix initialM = Matrix.identity(
                BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        final Matrix invInitialM = Utils.inverse(initialM);
        final Matrix initialGg = getInitialGg();
        final Matrix initialG = invInitialM.multiplyAndReturnNew(initialGg);

        final Matrix bg = getBiasAsMatrix();
        mB = invInitialM.multiplyAndReturnNew(bg);

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Before and after normalized gravity versors
                        return 2 * BodyKinematics.COMPONENTS;
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
                    public double evaluate(
                            final int i, final double[] point,
                            final double[] params, final double[] derivatives)
                            throws EvaluationException {
                        mI = i;

                        // point contains fixed gravity versor values for current
                        // sequence
                        mPoint = point;

                        gradientEstimator.gradient(params, derivatives);

                        return evaluateGeneralWithGDependentCrossBiases(i, params);
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

        // at this point covariance is expressed in terms of M and G, and must
        // be expressed in terms of Mg and Gg.
        // We know that:
        //Mg = M - I
        //Gg = M * G

        //M = [m11  m12  m13]
        //    [m21  m22  m23]
        //    [m31  m32  m33]

        //G = [g11  g12  g13]
        //    [g21  g22  g23]
        //    [g31  g32  g33]

        //Mg = [sx  mxy  mxz] = [m11 - 1    m12         m13     ]
        //     [myx	sy	 myz]   [m21        m22 - 1     m23     ]
        //     [mzx	mzy  sz ]   [m31        m32         m33 - 1 ]

        //Gg = [gg11  gg12  gg13] = [m11  m12  m13][g11  g12  g13]
        //     [gg21  gg22  gg23]   [m21  m22  m23][g21  g22  g23]
        //     [gg31  gg32  gg33]   [m31  m32  m33][g31  g32  g33]

        // Defining the linear application:
        // F(M, G) = F(m11, m21, m31, m12, m22, m32, m13, m23, m33, g11, g21, g31, g12, g22, g32, g13, g23, g33)
        // as:
        // [sx] =   [m11 - 1]
        // [sy]     [m22 - 1]
        // [sz]     [m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [m21]
        // [myz]    [m23]
        // [mzx]    [m31]
        // [mzy]    [m32]
        // [gg11]   [m11 * g11 + m12 * g21 + m13 * g31]
        // [gg21]   [m21 * g11 + m22 * g21 + m23 * g31]
        // [gg31]   [m31 * g11 + m32 * g21 + m33 * g31]
        // [gg12]   [m11 * g12 + m12 * g22 + m13 * g32]
        // [gg22]   [m21 * g12 + m22 * g22 + n23 * g32]
        // [gg32]   [m31 * g12 + m32 * g22 + m33 * g32]
        // [gg13]   [m11 * g13 + m12 * g23 + m13 * g33]
        // [gg23]   [m21 * g13 + m22 * g23 + m23 * g33]
        // [gg33]   [m31 * g13 + m32 * g23 + m33 * g33]

        // Then the Jacobian of F(M, G) is:
        //J = [1    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    1    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //    [g11  0    0    g21  0    0    g31  0    0    m11  m12  m13  0    0    0    0    0    0  ]
        //    [0    g11  0    0    g21  0    0    g31  0    m21  m22  m23  0    0    0    0    0    0  ]
        //    [0    0    g11  0    0    g21  0    0    g31  m31  m32  m33  0    0    0    0    0    0  ]
        //    [g12  0    0    g22  0    0    g32  0    0    0    0    0    m11  m12  m13  0    0    0  ]
        //    [0    g12  0    0    g22  0    0    g32  0    0    0    0    m21  m22  m23  0    0    0  ]
        //    [0    0    g12  0    0    g22  0    0    g32  0    0    0    m31  m32  m33  0    0    0  ]
        //    [g13  0    0    g23  0    0    g33  0    0    0    0    0    0    0    0    m11  m12  m13]
        //    [0    g13  0    0    g23  0    0    g33  0    0    0    0    0    0    0    m21  m22  m23]
        //    [0    0    g13  0    0    g23  0    0    g33  0    0    0    0    0    0    m31  m32  m33]

        // We know that the propagated covariance is J * Cov * J', hence:
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS_AND_CROSS_BIASES, GENERAL_UNKNOWNS_AND_CROSS_BIASES);

        jacobian.setElementAt(0, 0, 1.0);
        jacobian.setElementAt(1, 4, 1.0);
        jacobian.setElementAt(2, 8, 1.0);

        jacobian.setElementAt(3, 3, 1.0);
        jacobian.setElementAt(4, 6, 1.0);
        jacobian.setElementAt(5, 1, 1.0);

        jacobian.setElementAt(6, 7, 1.0);
        jacobian.setElementAt(7, 2, 1.0);
        jacobian.setElementAt(8, 5, 1.9);

        jacobian.setElementAt(9, 0, g11);
        jacobian.setElementAt(9, 3, g21);
        jacobian.setElementAt(9, 6, g31);
        jacobian.setElementAt(9, 9, m11);
        jacobian.setElementAt(9, 10, m12);
        jacobian.setElementAt(9, 11, m13);

        jacobian.setElementAt(10, 1, g11);
        jacobian.setElementAt(10, 4, g21);
        jacobian.setElementAt(10, 7, g31);
        jacobian.setElementAt(10, 9, m21);
        jacobian.setElementAt(10, 10, m22);
        jacobian.setElementAt(10, 11, m23);

        jacobian.setElementAt(11, 2, g11);
        jacobian.setElementAt(11, 5, g21);
        jacobian.setElementAt(11, 8, g31);
        jacobian.setElementAt(11, 9, m31);
        jacobian.setElementAt(11, 10, m32);
        jacobian.setElementAt(11, 11, m33);

        jacobian.setElementAt(12, 0, g12);
        jacobian.setElementAt(12, 3, g22);
        jacobian.setElementAt(12, 6, g32);
        jacobian.setElementAt(12, 12, m11);
        jacobian.setElementAt(12, 13, m12);
        jacobian.setElementAt(12, 14, m13);

        jacobian.setElementAt(13, 1, g12);
        jacobian.setElementAt(13, 4, g22);
        jacobian.setElementAt(13, 7, g32);
        jacobian.setElementAt(13, 12, m21);
        jacobian.setElementAt(13, 13, m22);
        jacobian.setElementAt(13, 14, m23);

        jacobian.setElementAt(14, 2, g12);
        jacobian.setElementAt(14, 5, g22);
        jacobian.setElementAt(14, 8, g32);
        jacobian.setElementAt(14, 12, m31);
        jacobian.setElementAt(14, 13, m32);
        jacobian.setElementAt(14, 14, m33);

        jacobian.setElementAt(15, 0, g13);
        jacobian.setElementAt(15, 3, g23);
        jacobian.setElementAt(15, 6, g33);
        jacobian.setElementAt(15, 15, m11);
        jacobian.setElementAt(15, 16, m12);
        jacobian.setElementAt(15, 17, m13);

        jacobian.setElementAt(16, 1, g13);
        jacobian.setElementAt(16, 4, g23);
        jacobian.setElementAt(16, 7, g33);
        jacobian.setElementAt(16, 15, m21);
        jacobian.setElementAt(16, 16, m22);
        jacobian.setElementAt(16, 17, m23);

        jacobian.setElementAt(17, 2, g13);
        jacobian.setElementAt(17, 5, g23);
        jacobian.setElementAt(17, 8, g33);
        jacobian.setElementAt(17, 15, m31);
        jacobian.setElementAt(17, 16, m32);
        jacobian.setElementAt(17, 17, m33);

        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed
     * for both the accelerometer and gyroscope and G-dependent cross biases
     * are ignored.
     *
     * @throws AlgebraException                         if accelerometer parameters prevent fixing
     *                                                  measured accelerometer values due to numerical instabilities.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateCommonAxis() throws AlgebraException,
            FittingException,
            com.irurueta.numerical.NotReadyException {
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

        final GradientEstimator gradientEstimator = new GradientEstimator(
                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(final double[] params)
                            throws EvaluationException {
                        return evaluateCommonAxis(mI, params);
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

        final Matrix bg = getBiasAsMatrix();
        mB = invInitialM.multiplyAndReturnNew(bg);

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Before and after normalized gravity versors
                        return 2 * BodyKinematics.COMPONENTS;
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
                    public double evaluate(
                            final int i, final double[] point,
                            final double[] params, final double[] derivatives)
                            throws EvaluationException {
                        mI = i;

                        // point contains fixed gravity versor values for current
                        // sequence
                        mPoint = point;

                        gradientEstimator.gradient(params, derivatives);

                        return evaluateCommonAxis(i, params);
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

        // at this point covariance is expressed in terms of M, and must
        // be expressed in terms of Mg.
        // We know that:
        //Mg = M - I
        //Gg = M * G

        //M = [m11  m12  m13]
        //    [0    m22  m23]
        //    [0    0    m33]

        //G = [g11  g12  g13] = 0
        //    [g21  g22  g23]
        //    [g31  g32  g33]

        //Mg = [sx  mxy  mxz] = [m11 - 1    m12         m13     ]
        //     [myx	sy	 myz]   [0          m22 - 1     m23     ]
        //     [mzx	mzy  sz ]   [0          0           m33 - 1 ]

        //Gg = [gg11  gg12  gg13] = [m11  m12  m13][g11  g12  g13]
        //     [gg21  gg22  gg23]   [0    m22  m23][g21  g22  g23]
        //     [gg31  gg32  gg33]   [0    0    m33][g31  g32  g33]

        // Defining the linear application:
        // F(M) = F(m11, m12, m22, m32=0, m13, m23, m33)
        // as:
        // [sx] =   [m11 - 1]
        // [sy]     [m22 - 1]
        // [sz]     [m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [0]
        // [myz]    [m23]
        // [mzx]    [0]
        // [mzy]    [0]
        // [gg11]   [0]
        // [gg21]   [0]
        // [gg31]   [0]
        // [gg12]   [0]
        // [gg22]   [0]
        // [gg32]   [0]
        // [gg13]   [0]
        // [gg23]   [0]
        // [gg33]   [0]

        // Then the Jacobian of F(M) is:
        //J = [1    0    0    0    0    0]
        //    [0    0    1    0    0    0]
        //    [0    0    0    0    0    1]
        //    [0    1    0    0    0    0]
        //    [0    0    0    1    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    1    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]
        //    [0    0    0    0    0    0]

        // We know that the propagated covariance is J * Cov * J', hence:
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS_AND_CROSS_BIASES, COMMON_Z_AXIS_UNKNOWNS);

        jacobian.setElementAt(0, 0, 1.0);
        jacobian.setElementAt(1, 2, 1.0);
        jacobian.setElementAt(2, 5, 1.0);

        jacobian.setElementAt(3, 1, 1.0);
        jacobian.setElementAt(4, 3, 1.0);

        jacobian.setElementAt(6, 4, 1.0);

        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
    }

    /**
     * Internal method to perform general calibration when G-dependant cross
     * biases are ignored.
     *
     * @throws AlgebraException                         if accelerometer parameters prevent fixing
     *                                                  measured accelerometer values due to numerical instabilities.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateGeneral()
            throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
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

        final GradientEstimator gradientEstimator = new GradientEstimator(
                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(final double[] params)
                            throws EvaluationException {
                        return evaluateGeneral(mI, params);
                    }
                });

        final Matrix initialM = Matrix.identity(
                BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        final Matrix invInitialM = Utils.inverse(initialM);

        final Matrix bg = getBiasAsMatrix();
        mB = invInitialM.multiplyAndReturnNew(bg);

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Before and after normalized gravity versors
                        return 2 * BodyKinematics.COMPONENTS;
                    }

                    @Override
                    public double[] createInitialParametersArray() {
                        return initialM.toArray();
                    }

                    @Override
                    public double evaluate(
                            final int i, final double[] point,
                            final double[] params, final double[] derivatives)
                            throws EvaluationException {
                        mI = i;

                        // point contains fixed gravity versor values for current
                        // sequence
                        mPoint = point;

                        gradientEstimator.gradient(params, derivatives);

                        return evaluateGeneral(i, params);
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

        // at this point covariance is expressed in terms of M, and must
        // be expressed in terms of Mg.
        // We know that:
        //Mg = M - I
        //Gg = M * G

        //M = [m11  m12  m13]
        //    [m21  m22  m23]
        //    [m31  m32  m33]

        //G = [g11  g12  g13] = 0
        //    [g21  g22  g23]
        //    [g31  g32  g33]

        //Mg = [sx  mxy  mxz] = [m11 - 1    m12         m13     ]
        //     [myx	sy	 myz]   [m21        m22 - 1     m23     ]
        //     [mzx	mzy  sz ]   [m31        m32         m33 - 1 ]

        //Gg = [gg11  gg12  gg13] = [m11  m12  m13][g11  g12  g13]
        //     [gg21  gg22  gg23]   [m21  m22  m23][g21  g22  g23]
        //     [gg31  gg32  gg33]   [m31  m32  m33][g31  g32  g33]

        // Defining the linear application:
        // F(M) = F(m11, m21, m31, m12, m22, m32, m13, m23, m33)
        // as:
        // [sx] =   [m11 - 1]
        // [sy]     [m22 - 1]
        // [sz]     [m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [m21]
        // [myz]    [m23]
        // [mzx]    [m31]
        // [mzy]    [m32]
        // [gg11]   [0]
        // [gg21]   [0]
        // [gg31]   [0]
        // [gg12]   [0]
        // [gg22]   [0]
        // [gg32]   [0]
        // [gg13]   [0]
        // [gg23]   [0]
        // [gg33]   [0]

        // Then the Jacobian of F(M) is:
        //J = [1    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    1    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    1]
        //    [0    0    0    1    0    0    0    0    0]
        //    [0    0    0    0    0    0    1    0    0]
        //    [0    1    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    1    0]
        //    [0    0    1    0    0    0    0    0    0]
        //    [0    0    0    0    0    1    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]
        //    [0    0    0    0    0    0    0    0    0]

        // We know that the propagated covariance is J * Cov * J', hence:
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS_AND_CROSS_BIASES, GENERAL_UNKNOWNS);

        jacobian.setElementAt(0, 0, 1.0);
        jacobian.setElementAt(1, 4, 1.0);
        jacobian.setElementAt(2, 8, 1.0);

        jacobian.setElementAt(3, 3, 1.0);
        jacobian.setElementAt(4, 6, 1.0);
        jacobian.setElementAt(5, 1, 1.0);

        jacobian.setElementAt(6, 7, 1.0);
        jacobian.setElementAt(7, 2, 1.0);
        jacobian.setElementAt(8, 5, 1.9);

        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
    }


    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws AlgebraException if there are numerical instabilities.
     */
    private void setInputData() throws AlgebraException {
        final Matrix ba = getAccelerometerBiasAsMatrix();
        final Matrix ma = getAccelerometerMa();

        final double[] measuredBeforeF = new double[BodyKinematics.COMPONENTS];
        final double[] fixedBeforeF = new double[BodyKinematics.COMPONENTS];

        final double[] measuredAfterF = new double[BodyKinematics.COMPONENTS];
        final double[] fixedAfterF = new double[BodyKinematics.COMPONENTS];

        final int numSequences = mSequences.size();
        final Matrix x = new Matrix(numSequences,
                2 * BodyKinematics.COMPONENTS);
        final double[] y = new double[numSequences];
        final double[] standardDeviations = new double[numSequences];

        // make a copy of input sequences that will be used to update
        // kinematics measurements with fixed values for memory efficiency

        mFixedSequences = new ArrayList<>();
        for (BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence : mSequences) {
            mFixedSequences.add(new BodyKinematicsSequence<>(sequence));
        }

        mAccelerationFixer.setBias(ba);
        mAccelerationFixer.setCrossCouplingErrors(ma);

        int i = 0;
        for (final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence : mSequences) {
            // sequence mean accelerometer samples of previous static
            // period will need to be fixed using accelerometer calibration
            // parameters
            measuredBeforeF[0] = sequence.getBeforeMeanFx();
            measuredBeforeF[1] = sequence.getBeforeMeanFy();
            measuredBeforeF[2] = sequence.getBeforeMeanFz();
            mAccelerationFixer.fix(measuredBeforeF, fixedBeforeF);

            measuredAfterF[0] = sequence.getAfterMeanFx();
            measuredAfterF[1] = sequence.getAfterMeanFy();
            measuredAfterF[2] = sequence.getAfterMeanFz();
            mAccelerationFixer.fix(measuredAfterF, fixedAfterF);

            // because we are only interested in gravity direction, we
            // normalize these vectors, so that gravity becomes independent
            // of current Earth position.
            ArrayUtils.normalize(fixedBeforeF);
            ArrayUtils.normalize(fixedAfterF);

            x.setSubmatrix(i, 0, i, 2,
                    fixedBeforeF);
            x.setSubmatrix(i, 3, i, 5,
                    fixedAfterF);

            y[i] = 0.0;

            standardDeviations[i] = computeAverageAngularRateStandardDeviation(
                    sequence);
            i++;
        }

        mFitter.setInputData(x, y, standardDeviations);
    }

    /**
     * Computes average angular rate standard deviation for measurements
     * in provided sequence.
     *
     * @param sequence a sequence.
     * @return average angular rate standard deviation expressed in radians
     * per second (rad/s).
     */
    private static double computeAverageAngularRateStandardDeviation(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence) {
        final List<StandardDeviationTimedBodyKinematics> items = sequence.getSortedItems();
        final double size = items.size();

        double result = 0.0;
        for (final StandardDeviationTimedBodyKinematics item : items) {
            result += item.getAngularRateStandardDeviation() / size;
        }

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
     * Converts angular speed instance to radians per second (rad/s).
     *
     * @param value angular speed value.
     * @param unit unit of angular speed value.
     * @return converted value.
     */
    private static double convertAngularSpeed(final double value, final AngularSpeedUnit unit) {
        return AngularSpeedConverter.convert(value, unit,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Converts angular speed instance to radians per second (rad/s).
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return convertAngularSpeed(angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit());
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
        mEstimatedMse = mFitter.getMse();
    }

    /**
     * Computes gravity versor error at the end of a sequence using provided
     * parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i      row position.
     * @param params array containing parameters for the general purpose case
     *               when G-dependent cross biases are taken into account. Must
     *               have length 18.
     * @return error between estimated and measured gravity versor.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateGeneralWithGDependentCrossBiases(
            final int i, final double[] params)
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

        return evaluate(i, m11, m21, m31, m12, m22, m32,
                m13, m23, m33, g11, g21, g31, g12, g22, g32,
                g13, g23, g33);
    }

    /**
     * Computes gravity versor error at the end of a sequence using provided
     * parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i      row position.
     * @param params array containing parameters for the general purpose case
     *               when G-dependent cross biases are taken into account. Must
     *               have length 15.
     * @return error between estimated and measured gravity versor.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateCommonAxisWithGDependentCrossBiases(
            final int i, final double[] params)
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

        return evaluate(i, m11, 0.0, 0.0, m12, m22, 0.0,
                m13, m23, m33, g11, g21, g31, g12, g22, g32,
                g13, g23, g33);
    }

    /**
     * Computes gravity versor error at the end of a sequence using provided
     * parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i      row position.
     * @param params array containing current parameters for the general purpose case
     *               when G-dependent cross biases are ignored. Must have length 9.
     * @return error between estimated and measured gravity versor.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateGeneral(
            final int i, final double[] params)
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

        return evaluate(i, m11, m21, m31, m12, m22, m32,
                m13, m23, m33, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);
    }

    /**
     * Computes gravity versor error at the end of a sequence using provided
     * parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i      row position.
     * @param params array containing current parameters for the common z-axis case
     *               when G-dependent cross biases are ignored. Must have length 9.
     * @return error between estimated and measured gravity versor.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateCommonAxis(
            final int i, final double[] params)
            throws EvaluationException {
        final double m11 = params[0];

        final double m12 = params[1];
        final double m22 = params[2];

        final double m13 = params[3];
        final double m23 = params[4];
        final double m33 = params[5];

        return evaluate(i, m11, 0.0, 0.0, m12, m22, 0.0,
                m13, m23, m33, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);
    }

    /**
     * Computes gravity versor error at the end of a sequence using provided
     * parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param i   row position.
     * @param m11 element 1,1 of cross-coupling error matrix.
     * @param m21 element 2,1 of cross-coupling error matrix.
     * @param m31 element 3,1 of cross-coupling error matrix.
     * @param m12 element 1,2 of cross-coupling error matrix.
     * @param m22 element 2,2 of cross-coupling error matrix.
     * @param m32 element 3,2 of cross-coupling error matrix.
     * @param m13 element 1,3 of cross-coupling error matrix.
     * @param m23 element 2,3 of cross-coupling error matrix.
     * @param m33 element 3,3 of cross-coupling error matrix.
     * @param g11 element 1,1 of g-dependent cross bias matrix.
     * @param g21 element 2,1 of g-dependent cross bias matrix.
     * @param g31 element 3,1 of g-dependent cross bias matrix.
     * @param g12 element 1,2 of g-dependent cross bias matrix.
     * @param g22 element 2,2 of g-dependent cross bias matrix.
     * @param g32 element 3,2 of g-dependent cross bias matrix.
     * @param g13 element 1,3 of g-dependent cross bias matrix.
     * @param g23 element 2,3 of g-dependent cross bias matrix.
     * @param g33 element 3,3 of g-dependent cross bias matrix.
     * @return error between estimated and measured gravity versor.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluate(
            final int i,
            final double m11, final double m21, final double m31,
            final double m12, final double m22, final double m32,
            final double m13, final double m23, final double m33,
            final double g11, final double g21, final double g31,
            final double g12, final double g22, final double g32,
            final double g13, final double g23, final double g33)
            throws EvaluationException {
        try {
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measuredSequence =
                    mSequences.get(i);
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> fixedSequence =
                    mFixedSequences.get(i);

            // generate new sequence using current parameters to fix angular rate measurements
            if (mMeasuredSpecificForce == null) {
                mMeasuredSpecificForce = new Matrix(
                        BodyKinematics.COMPONENTS, 1);
            }
            if (mTrueSpecificForce == null) {
                mTrueSpecificForce = new Matrix(
                        BodyKinematics.COMPONENTS, 1);
            }

            if (mMeasuredAngularRate == null) {
                mMeasuredAngularRate = new Matrix(
                        BodyKinematics.COMPONENTS, 1);
            }
            if (mTrueAngularRate == null) {
                mTrueAngularRate = new Matrix(
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

            if (mG == null) {
                mG = new Matrix(BodyKinematics.COMPONENTS,
                        BodyKinematics.COMPONENTS);
            }
            if (mTmp == null) {
                mTmp = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

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

            mG.setElementAt(0, 0, g11);
            mG.setElementAt(1, 0, g21);
            mG.setElementAt(2, 0, g31);

            mG.setElementAt(0, 1, g12);
            mG.setElementAt(1, 1, g22);
            mG.setElementAt(2, 1, g32);

            mG.setElementAt(0, 2, g13);
            mG.setElementAt(1, 2, g23);
            mG.setElementAt(2, 2, g33);

            // fix kinematics
            final int numItems = measuredSequence.getItemsCount();
            final List<StandardDeviationTimedBodyKinematics> measuredItems = measuredSequence.getSortedItems();
            final List<StandardDeviationTimedBodyKinematics> fixedItems = fixedSequence.getSortedItems();
            for (int j = 0; j < numItems; j++) {
                final StandardDeviationTimedBodyKinematics measuredItem = measuredItems.get(j);
                final StandardDeviationTimedBodyKinematics fixedItem = fixedItems.get(j);

                fixKinematics(measuredItem.getKinematics(), fixedItem.getKinematics());
            }


            // integrate fixed sequence to obtain attitude change
            QuaternionIntegrator.integrateGyroSequence(fixedSequence, mQ);

            mStartPoint.setInhomogeneousCoordinates(
                    mPoint[0], mPoint[1], mPoint[2]);
            mQ.inverse();
            mQ.rotate(mStartPoint, mEndPoint);

            mExpectedEndPoint.setInhomogeneousCoordinates(
                    mPoint[3], mPoint[4], mPoint[5]);

            return mExpectedEndPoint.distanceTo(mEndPoint);

        } catch (final AlgebraException e) {
            throw new EvaluationException(e);
        }
    }

    /**
     * Fixes provided kinematics with provided accelerometer parameters and
     * current gyroscope parameters.
     *
     * @param kinematics kinematics to be fixed with current values.
     * @param result     kinematics where result will be stored.
     * @throws AlgebraException if for some reason kinematics
     */
    private void fixKinematics(final BodyKinematics kinematics, final BodyKinematics result)
            throws AlgebraException {

        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue
        // Ωmeas = M*(Ωtrue + b + G * ftrue)

        // M = I + Mg
        // bg = M*b --> b = M^-1*bg
        // Gg = M*G --> G = M^-1*Gg

        // Ωtrue = M^-1 * Ωmeas - b - G*ftrue

        // fix specific force
        mMeasuredSpecificForce.setElementAtIndex(0, kinematics.getFx());
        mMeasuredSpecificForce.setElementAtIndex(1, kinematics.getFy());
        mMeasuredSpecificForce.setElementAtIndex(2, kinematics.getFz());

        mAccelerationFixer.fix(mMeasuredSpecificForce, mTrueSpecificForce);

        // fix angular rate
        mMeasuredAngularRate.setElementAtIndex(0, kinematics.getAngularRateX());
        mMeasuredAngularRate.setElementAtIndex(1, kinematics.getAngularRateY());
        mMeasuredAngularRate.setElementAtIndex(2, kinematics.getAngularRateZ());

        mG.multiply(mTrueSpecificForce, mTmp);
        mInvM.multiply(mMeasuredAngularRate, mTrueAngularRate);
        mTrueAngularRate.subtract(mB);
        mTrueAngularRate.subtract(mTmp);

        result.setSpecificForceCoordinates(
                mTrueSpecificForce.getElementAtIndex(0),
                mTrueSpecificForce.getElementAtIndex(1),
                mTrueSpecificForce.getElementAtIndex(2));

        result.setAngularRateCoordinates(
                mTrueAngularRate.getElementAtIndex(0),
                mTrueAngularRate.getElementAtIndex(1),
                mTrueAngularRate.getElementAtIndex(2));
    }

    /**
     * Resets any previous estimations.
     */
    private void reset() {
        mEstimatedMg = null;
        mEstimatedGg = null;
        mEstimatedCovariance = null;
        mEstimatedChiSq = 0.0;
        mEstimatedMse = 0.0;
        mI = 0;
        mFixedSequences = null;
        mMeasuredSpecificForce = null;
        mTrueSpecificForce = null;
        mMeasuredAngularRate = null;
        mTrueAngularRate = null;
        mM = null;
        mInvM = null;
        mB = null;
        mG = null;
        mTmp = null;
        mPoint = null;
    }
}
