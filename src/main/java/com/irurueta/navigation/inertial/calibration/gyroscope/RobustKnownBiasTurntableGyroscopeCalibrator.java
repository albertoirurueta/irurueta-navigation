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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.AxisRotation3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate gyroscope
 * cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer when gyroscope biases are known.
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
public abstract class RobustKnownBiasTurntableGyroscopeCalibrator {
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
     * Default turntable rotation rate.
     */
    public static final double DEFAULT_TURNTABLE_ROTATION_RATE =
            TurntableGyroscopeCalibrator.DEFAULT_TURNTABLE_ROTATION_RATE;

    /**
     * Default time interval between measurements expressed in seconds (s).
     * This is a typical value when we have 50 samples per second.
     */
    public static final double DEFAULT_TIME_INTERVAL =
            TurntableGyroscopeCalibrator.DEFAULT_TIME_INTERVAL;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.LMedS;

    /**
     * Indicates that result is refined by default using a non-linear calibrator
     * (which uses a Levenberg-Marquardt fitter).
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = true;

    /**
     * Default amount of progress variation before notifying a change in estimation progress.
     * By default this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;

    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;

    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;

    /**
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen subsamples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;

    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;

    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;

    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;

    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;

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
     * Constant rotation rate at which the turntable is spinning.
     * This is expressed in radians per second (rad/s).
     */
    private double mTurntableRotationRate = DEFAULT_TURNTABLE_ROTATION_RATE;

    /**
     * Time interval between measurements being captured expressed in
     * second (s).
     */
    private double mTimeInterval = DEFAULT_TIME_INTERVAL;

    /**
     * Contains a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     */
    protected List<StandardDeviationBodyKinematics> mMeasurements;

    /**
     * Position where body kinematics measures have been taken.
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
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownBiasTurntableGyroscopeCalibratorListener mListener;

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
     * Estimated mean square error respect to provided measurements.
     */
    private double mEstimatedMse;

    /**
     * Indicates whether calibrator is running.
     */
    protected boolean mRunning;

    /**
     * Amount of progress variation before notifying a progress change during calibration.
     */
    protected float mProgressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is equivalent
     * to 100%). The amount of confidence indicates the probability that the estimated
     * result is correct. Usually this value will be close to 1.0, but not exactly 1.0.
     */
    protected double mConfidence = DEFAULT_CONFIDENCE;

    /**
     * Maximum allowed number of iterations. When the maximum number of iterations is
     * exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     */
    protected int mMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Data related to inliers found after calibration.
     */
    protected InliersData mInliersData;

    /**
     * Indicates whether result must be refined using a non linear calibrator over
     * found inliers.
     * If true, inliers will be computed and kept in any implementation regardless of the
     * settings.
     */
    protected boolean mRefineResult = DEFAULT_REFINE_RESULT;

    /**
     * Size of subsets to be checked during robust estimation.
     */
    protected int mPreliminarySubsetSize = TurntableGyroscopeCalibrator
            .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean mKeepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Inner non-robust calibrator.
     */
    private final KnownBiasTurntableGyroscopeCalibrator mInnerCalibrator =
            new KnownBiasTurntableGyroscopeCalibrator();

    /**
     * Constructor.
     */
    public RobustKnownBiasTurntableGyroscopeCalibrator() {
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this();
        mPosition = position;
        mMeasurements = measurements;
        try {
            setTurntableRotationRate(turntableRotationRate);
            setTimeInterval(timeInterval);
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg);
        mListener = listener;
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
     * @param bias                  known gyroscope bias. This must have
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this();
        mPosition = position;
        mMeasurements = measurements;
        try {
            setTurntableRotationRate(turntableRotationRate);
            setTimeInterval(timeInterval);
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
     * @param bias                  known gyroscope bias. This must have length
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg);
        mListener = listener;
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
     * @param bias                  known gyroscope bias. This must have length
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg);
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
     * @param bias                  known gyroscope bias. This must have length
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        mListener = listener;
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg);
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        mListener = listener;
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
     * @param bias                          known gyroscope. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, bias,
                initialMg, initialGg);
        mListener = listener;
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, bias,
                initialMg, initialGg);
        mListener = listener;
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
     * @param bias                          known gyroscope bias. This must
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, bias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
        mListener = listener;
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, turntableRotationRate, timeInterval, measurements,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
     * @param bias                          known gyroscope bias. This must be
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements,
                commonAxisUsed, estimateGDependentCrossBiases, bias,
                initialMg, initialGg, accelerometerBias, accelerometerMa);
        mListener = listener;
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg);
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, listener);
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
     * @param bias                  known gyroscope bias. This must have
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg);
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
     * @param bias                  known gyroscope bias. This must have length
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, listener);
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
     * @param bias                  known gyroscope bias. This must have length
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
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
     * @param bias                  known gyroscope bias. This must have length
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, listener);
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, listener);
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener);
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg);
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener);
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
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
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
     * @param bias                          known gyroscope bias. This must
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias,
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
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          known gyroscope bias. This
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
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
     * @param bias                          known gyroscope bias. This must be
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
    public RobustKnownBiasTurntableGyroscopeCalibrator(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener);
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
     * Sets known y-coordinate of gyroscope bias.
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
     * Gets known z-coordinate of gyroscope bias.
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
     * Sets known bias coordinates of gyroscope
     * expressed in radians per second (rad/s).
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
     * Sets known bias coordinates of gyroscope used to find a solution.
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
     * Gets known gyroscope bias.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of known gyroscope bias.
     */
    public double[] getBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets known gyroscope bias.
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
     * Sets known gyroscope bias.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known bias to find a solution.
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
     * Gets constant rotation rate at which the turntable is spinning.
     * This is expressed in radians per second (rad/s).
     *
     * @return constant rotation rate of turntable.
     */
    public double getTurntableRotationRate() {
        return mTurntableRotationRate;
    }

    /**
     * Sets constant rotation rate at which the turntable is spinning.
     * This is expressed in radians per second (rad/s).
     *
     * @param turntableRotationRate constant rotation rate of turntable.
     * @throws LockedException          if calibrator is currently running
     * @throws IllegalArgumentException if provided value is zero or
     *                                  negative.
     */
    public void setTurntableRotationRate(
            final double turntableRotationRate) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (turntableRotationRate <= 0.0) {
            throw new IllegalArgumentException();
        }

        mTurntableRotationRate = turntableRotationRate;
    }

    /**
     * Gets constant rotation rate at which the turntable is spinning.
     *
     * @return constant rotation rate of turntable.
     */
    public AngularSpeed getTurntableRotationRateAsAngularSpeed() {
        return new AngularSpeed(mTurntableRotationRate,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets constant rotation rate at which the turntable is spinning.
     *
     * @param result instance where result will be stored.
     */
    public void getTurntableRotationRateAsAngularSpeed(
            final AngularSpeed result) {
        result.setValue(mTurntableRotationRate);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets constant rotation rate at which the turntable is spinning.
     *
     * @param turntableRotationRate constant rotation rate of turntable.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is zero or
     *                                  negative.
     */
    public void setTurntableRotationRate(
            final AngularSpeed turntableRotationRate)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        setTurntableRotationRate(
                convertAngularSpeed(turntableRotationRate));
    }

    /**
     * Gets time interval between measurements being captured expressed in
     * seconds (s).
     *
     * @return time interval between measurements.
     */
    public double getTimeInterval() {
        return mTimeInterval;
    }

    /**
     * Sets time interval between measurements being captured expressed in
     * seconds (s).
     *
     * @param timeInterval time interval between measurements.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is zero or
     *                                  negative.
     */
    public void setTimeInterval(final double timeInterval)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (timeInterval <= 0.0) {
            throw new IllegalArgumentException();
        }
        mTimeInterval = timeInterval;
    }

    /**
     * Gets time interval between measurements being captured.
     *
     * @return time interval between measurements.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(mTimeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between measurements being captured.
     *
     * @param result instance where result will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(mTimeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between measurements being captured.
     *
     * @param timeInterval time interval between measurements.
     * @throws LockedException if calibrator is currently running.
     */
    public void setTimeInterval(final Time timeInterval)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        setTimeInterval(convertTime(timeInterval));
    }

    /**
     * Gets a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     *
     * @return collection of body kinematics measurements at a known position
     * with unknown orientations.
     */
    public List<StandardDeviationBodyKinematics> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     *
     * @param measurements collection of body kinematics measurements at a
     *                     known position witn unknown orientations.
     * @throws LockedException if calibrator is currently running.
     */
    public void setMeasurements(
            final List<StandardDeviationBodyKinematics> measurements)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMeasurements = measurements;
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
    public void setPosition(final ECEFPosition position)
            throws LockedException {
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
    public void setPosition(final NEDPosition position)
            throws LockedException {
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
    public void setCommonAxisUsed(final boolean commonAxisUsed)
            throws LockedException {
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
    public RobustKnownBiasTurntableGyroscopeCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets minimum number of required measurements.
     *
     * @return minimum number of required measurements.
     */
    public int getMinimumRequiredMeasurements() {
        if (mCommonAxisUsed) {
            if (mEstimateGDependentCrossBiases) {
                return KnownBiasTurntableGyroscopeCalibrator
                        .MINIMUM_MEASUREMENTS_COMMON_Z_AXIS_AND_CROSS_BIASES;
            } else {
                return KnownBiasTurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS;
            }
        } else {
            if (mEstimateGDependentCrossBiases) {
                return KnownBiasTurntableGyroscopeCalibrator
                        .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES;
            } else {
                return KnownBiasTurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            }
        }
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    public boolean isReady() {
        return mMeasurements != null
                && mMeasurements.size() >= getMinimumRequiredMeasurements();
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
     * Returns amount of progress variation before notifying a progress change during
     * calibration.
     *
     * @return amount of progress variation before notifying a progress change during
     * calibration.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * calibration.
     *
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during calibration.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setProgressDelta(final float progressDelta)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA ||
                progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     *
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return mConfidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability that
     * the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setConfidence(final double confidence)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mConfidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number of
     * iterations is achieved without converging to a result when calling calibrate(),
     * a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of iterations
     * is exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setMaxIterations(final int maxIterations)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mMaxIterations = maxIterations;
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mInliersData;
    }

    /**
     * Indicates whether result must be refined using a non-linear solver over found inliers.
     *
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result must be refined using a non-linear solver over found inliers.
     *
     * @param refineResult true to refine result, false to simply use result found by robust
     *                     estimator without further refining.
     * @throws LockedException if calibrator is currently running.
     */
    public void setResultRefined(final boolean refineResult)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mRefineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return mKeepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining result,
     *                       false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    public void setCovarianceKept(final boolean keepCovariance)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mKeepCovariance = keepCovariance;
    }

    /**
     * Returns quality scores corresponding to each measurement.
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @return quality scores corresponding to each sample.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each measurement.
     * The larger the score value the better the quality of the sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setQualityScores(final double[] qualityScores)
            throws LockedException {
    }

    /**
     * Estimates gyroscope calibration parameters containing bias, scale factors,
     * cross-coupling errors and G-dependent coupling.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    public abstract void calibrate() throws LockedException, NotReadyException,
            CalibrationException;

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
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32, gg13, gg23, gg33.
     *
     * @return estimated covariance matrix for estimated parameters.
     */
    public Matrix getEstimatedCovariance() {
        return mEstimatedCovariance;
    }

    /**
     * Gets estimated mean square error respect to provided measurements.
     *
     * @return estimated mean square error respect to provided measurements.
     */
    public double getEstimatedMse() {
        return mEstimatedMse;
    }

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinimumRequiredMeasurements()}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return mPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinimumRequiredMeasurements}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is less than
     *                                  {@link #getMinimumRequiredMeasurements}.
     */
    public void setPreliminarySubsetSize(final int preliminarySubsetSize)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < getMinimumRequiredMeasurements()) {
            throw new IllegalArgumentException();
        }

        mPreliminarySubsetSize = preliminarySubsetSize;
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param method robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator();
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator();
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator();
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator();
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This must
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This must be
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(position,
                        turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This must
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This must be
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator();
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator();
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final double turntableRotationRate, final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias, final Matrix initialMg,
            final Matrix initialGg, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg, initialGg,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg, initialGg,
                        listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This must
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, commonAxisUsed,
                        estimateGDependentCrossBiases, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This must be
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg,
                        accelerometerBias, accelerometerMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate,
                        timeInterval, measurements, bias, initialMg,
                        initialGg, accelerometerBias, accelerometerMa,
                        listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @param method                robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg,
                        initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @param bias                          known gyroscope bias. This must be
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
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size, if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative or
     *                                  if provided quality scores length is
     *                                  smaller than 7 samples.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        position, turntableRotationRate, timeInterval, measurements,
                        commonAxisUsed, estimateGDependentCrossBiases, bias,
                        initialMg, initialGg, accelerometerBias, accelerometerMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasTurntableGyroscopeCalibrator(
                        qualityScores, position, turntableRotationRate, timeInterval,
                        measurements, commonAxisUsed, estimateGDependentCrossBiases,
                        bias, initialMg, initialGg, accelerometerBias,
                        accelerometerMa, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This must
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This must be
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final ECEFPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must have length
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                  known gyroscope bias. This must be 3x1 and
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed,
                estimateGDependentCrossBiases, bias, initialMg,
                initialGg, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This must
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final double[] bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final double[] accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @param bias                          known gyroscope bias. This must be
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
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public static RobustKnownBiasTurntableGyroscopeCalibrator create(
            final NEDPosition position,
            final double turntableRotationRate,
            final double timeInterval,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases,
            final Matrix bias,
            final Matrix initialMg,
            final Matrix initialGg,
            final Matrix accelerometerBias,
            final Matrix accelerometerMa,
            final RobustKnownBiasTurntableGyroscopeCalibratorListener listener) {
        return create(position, turntableRotationRate, timeInterval,
                measurements, commonAxisUsed, estimateGDependentCrossBiases,
                bias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener, DEFAULT_ROBUST_METHOD);
    }


    /**
     * Computes error of a preliminary result respect a given measurement.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final StandardDeviationBodyKinematics measurement,
            final PreliminaryResult preliminaryResult) {
        // We know that measured angular rate is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [myx   sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [mzx   mzy    sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        final BodyKinematics measuredKinematics = measurement.getKinematics();

        final double[] specificForce = new double[]{
                measuredKinematics.getFx(),
                measuredKinematics.getFy(),
                measuredKinematics.getFz()
        };

        try {
            final double[] axis1 = ArrayUtils.normalizeAndReturnNew(specificForce);
            final Quaternion rot1 = new Quaternion(axis1, 0.0);

            final CoordinateTransformation nedC1 = new CoordinateTransformation(
                    rot1.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDPosition nedPosition = getNedPosition();
            final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
            final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame1);
            double timeInterval = mTimeInterval;
            double angleIncrement = mTurntableRotationRate * timeInterval;
            if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                // angle = rot_rate * interval
                // rot_rate * interval / x = angle / x

                // if we want angle / x = pi / 2, then:
                final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                timeInterval /= x;
                angleIncrement = mTurntableRotationRate * timeInterval;
            }
            final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
            final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
            final CoordinateTransformation nedC2 =
                    new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame2);

            final BodyKinematics expectedKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                            ecefFrame1);

            final double angularRateMeasX1 = measuredKinematics.getAngularRateX();
            final double angularRateMeasY1 = measuredKinematics.getAngularRateY();
            final double angularRateMeasZ1 = measuredKinematics.getAngularRateZ();

            final double angularRateTrueX = expectedKinematics.getAngularRateX();
            final double angularRateTrueY = expectedKinematics.getAngularRateY();
            final double angularRateTrueZ = expectedKinematics.getAngularRateZ();

            final double fTrueX = expectedKinematics.getFx();
            final double fTrueY = expectedKinematics.getFy();
            final double fTrueZ = expectedKinematics.getFz();

            final Matrix mg = preliminaryResult.mEstimatedMg;

            final Matrix gg = preliminaryResult.mEstimatedGg;

            final Matrix m1 = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            m1.add(mg);

            final Matrix angularRateTrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            angularRateTrue.setElementAtIndex(0, angularRateTrueX);
            angularRateTrue.setElementAtIndex(1, angularRateTrueY);
            angularRateTrue.setElementAtIndex(2, angularRateTrueZ);

            m1.multiply(angularRateTrue);

            final Matrix fTrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            fTrue.setElementAtIndex(0, fTrueX);
            fTrue.setElementAtIndex(1, fTrueY);
            fTrue.setElementAtIndex(2, fTrueZ);
            final Matrix m2 = gg.multiplyAndReturnNew(fTrue);

            m1.add(m2);

            final double angularRateMeasX2 = mBiasX + m1.getElementAtIndex(0);
            final double angularRateMeasY2 = mBiasY + m1.getElementAtIndex(1);
            final double angularRateMeasZ2 = mBiasZ + m1.getElementAtIndex(2);

            final double sqrNormMeas1 = angularRateMeasX1 * angularRateMeasX1
                    + angularRateMeasY1 * angularRateMeasY1
                    + angularRateMeasZ1 * angularRateMeasZ1;
            final double sqrNormMeas2 = angularRateMeasX2 * angularRateMeasX2
                    + angularRateMeasY2 * angularRateMeasY2
                    + angularRateMeasZ2 * angularRateMeasZ2;

            final double normMeas1 = Math.sqrt(sqrNormMeas1);
            final double normMeas2 = Math.sqrt(sqrNormMeas2);

            return Math.abs(normMeas1 - normMeas2);

        } catch (final WrongSizeException | InvalidRotationMatrixException
                | InvalidSourceAndDestinationFrameTypeException e) {
            return Double.MAX_VALUE;
        }
    }

    /**
     * Computes a preliminary solution for a subset of samples picked by a robust estimator.
     *
     * @param samplesIndices indices of samples picked by the robust estimator.
     * @param solutions      list where estimated preliminary solution will be stored.
     */
    protected void computePreliminarySolutions(
            final int[] samplesIndices,
            final List<PreliminaryResult> solutions) {

        final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();

        for (final int samplesIndex : samplesIndices) {
            measurements.add(mMeasurements.get(samplesIndex));
        }

        try {
            final PreliminaryResult result = new PreliminaryResult();
            result.mEstimatedMg = getInitialMg();
            result.mEstimatedGg = getInitialGg();

            mInnerCalibrator.setTurntableRotationRate(mTurntableRotationRate);
            mInnerCalibrator.setTimeInterval(mTimeInterval);
            mInnerCalibrator.setGDependentCrossBiasesEstimated(mEstimateGDependentCrossBiases);
            mInnerCalibrator.setInitialMg(result.mEstimatedMg);
            mInnerCalibrator.setInitialGg(result.mEstimatedGg);
            mInnerCalibrator.setAccelerometerBias(mAccelerometerBiasX, mAccelerometerBiasY, mAccelerometerBiasZ);
            mInnerCalibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                    mAccelerometerSx, mAccelerometerSy, mAccelerometerSz,
                    mAccelerometerMxy, mAccelerometerMxz,
                    mAccelerometerMyx, mAccelerometerMyz,
                    mAccelerometerMzx, mAccelerometerMzy);
            mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
            mInnerCalibrator.setMeasurements(measurements);
            mInnerCalibrator.setPosition(mPosition);
            mInnerCalibrator.calibrate();

            result.mEstimatedMg = mInnerCalibrator.getEstimatedMg();
            result.mEstimatedGg = mInnerCalibrator.getEstimatedGg();

            if (mKeepCovariance) {
                result.mCovariance = mInnerCalibrator.getEstimatedCovariance();
            } else {
                result.mCovariance = null;
            }

            result.mEstimatedMse = mInnerCalibrator.getEstimatedMse();

            solutions.add(result);
        } catch (final LockedException | CalibrationException | NotReadyException e) {
            solutions.clear();
        }
    }

    /**
     * Attempts to refine calibration parameters if refinement is requested.
     * This method returns a refined solution or provided input if refinement is not
     * requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this method
     * will also keep covariance of refined position.
     *
     * @param preliminaryResult a preliminary result.
     */
    protected void attemptRefine(final PreliminaryResult preliminaryResult) {
        if (mRefineResult && mInliersData != null) {
            final BitSet inliers = mInliersData.getInliers();
            final int nSamples = mMeasurements.size();

            final List<StandardDeviationBodyKinematics> inlierMeasurements =
                    new ArrayList<>();
            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierMeasurements.add(mMeasurements.get(i));
                }
            }

            try {
                mInnerCalibrator.setTurntableRotationRate(mTurntableRotationRate);
                mInnerCalibrator.setTimeInterval(mTimeInterval);
                mInnerCalibrator.setGDependentCrossBiasesEstimated(mEstimateGDependentCrossBiases);
                mInnerCalibrator.setInitialMg(preliminaryResult.mEstimatedMg);
                mInnerCalibrator.setInitialGg(preliminaryResult.mEstimatedGg);
                mInnerCalibrator.setAccelerometerBias(mAccelerometerBiasX, mAccelerometerBiasY, mAccelerometerBiasZ);
                mInnerCalibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                        mAccelerometerSx, mAccelerometerSy, mAccelerometerSz,
                        mAccelerometerMxy, mAccelerometerMxz,
                        mAccelerometerMyx, mAccelerometerMyz,
                        mAccelerometerMzx, mAccelerometerMzy);
                mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mInnerCalibrator.setMeasurements(inlierMeasurements);
                mInnerCalibrator.setPosition(mPosition);
                mInnerCalibrator.calibrate();

                mEstimatedMg = mInnerCalibrator.getEstimatedMg();
                mEstimatedGg = mInnerCalibrator.getEstimatedGg();

                if (mKeepCovariance) {
                    mEstimatedCovariance = mInnerCalibrator.getEstimatedCovariance();
                } else {
                    mEstimatedCovariance = null;
                }

                mEstimatedMse = mInnerCalibrator.getEstimatedMse();

            } catch (LockedException | CalibrationException | NotReadyException e) {
                mEstimatedCovariance = preliminaryResult.mCovariance;
                mEstimatedMg = preliminaryResult.mEstimatedMg;
                mEstimatedGg = preliminaryResult.mEstimatedGg;
                mEstimatedMse = preliminaryResult.mEstimatedMse;
            }
        } else {
            mEstimatedCovariance = preliminaryResult.mCovariance;
            mEstimatedMg = preliminaryResult.mEstimatedMg;
            mEstimatedGg = preliminaryResult.mEstimatedGg;
            mEstimatedMse = preliminaryResult.mEstimatedMse;
        }
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
     * Converts time instance to seconds.
     *
     * @param time time instance to be converted.
     * @return converted value.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Internal class containing estimated preliminary result.
     */
    protected static class PreliminaryResult {
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
         * Covariance matrix for estimated result.
         */
        private Matrix mCovariance;

        /**
         * Estimated Mean Square Error.
         */
        private double mEstimatedMse;
    }
}
