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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate accelerometer cross couplings
 * and scaling factors.
 * <p>
 * To use this calibrator at least 7 measurements taken at a single known position must
 * be taken at 7 different unknown orientations and zero velocity when common z-axis
 * is assumed, otherwise at least 10 measurements are required.
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
public abstract class RobustKnownBiasAndPositionAccelerometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements when common z-axis is assumed.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            KnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMON_Z_AXIS;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL =
            KnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.LMedS;

    /**
     * Indicates that result is refined by default.
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
     * Contains a list of body kinematics measurements taken at a
     * given position with different unknown orientations and containing the
     * standard deviations of accelerometer and gyroscope measurements.
     */
    protected List<StandardDeviationBodyKinematics> mMeasurements;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownBiasAndPositionAccelerometerCalibratorListener mListener;

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
    protected int mPreliminarySubsetSize = MINIMUM_MEASUREMENTS_GENERAL;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * X-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mBiasX;

    /**
     * Y-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mBiasY;

    /**
     * Z-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mBiasZ;

    /**
     * Initial x scaling factor.
     */
    private double mInitialSx;

    /**
     * Initial y scaling factor.
     */
    private double mInitialSy;

    /**
     * Initial z scaling factor.
     */
    private double mInitialSz;

    /**
     * Initial x-y cross coupling error.
     */
    private double mInitialMxy;

    /**
     * Initial x-z cross coupling error.
     */
    private double mInitialMxz;

    /**
     * Initial y-x cross coupling error.
     */
    private double mInitialMyx;

    /**
     * Initial y-z cross coupling error.
     */
    private double mInitialMyz;

    /**
     * Initial z-x cross coupling error.
     */
    private double mInitialMzx;

    /**
     * Initial z-y cross coupling error.
     */
    private double mInitialMzy;

    /**
     * Position where body kinematics measures have been taken.
     */
    private ECEFPosition mPosition;

    /**
     * Estimated accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     */
    private Matrix mEstimatedMa;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean mKeepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Estimated covariance of estimated position.
     * This is only available when result has been refined and covariance is kept.
     */
    private Matrix mEstimatedCovariance;

    /**
     * Estimated mean square error respect to provided measurements.
     */
    private double mEstimatedMse;

    /**
     * Inner calibrator to compute calibration for each subset of data or during
     * final refining.
     */
    private final KnownBiasAndPositionAccelerometerCalibrator mInnerCalibrator =
            new KnownBiasAndPositionAccelerometerCalibrator();

    /**
     * Contains gravity norm for current position to be reused during calibration.
     */
    protected double mGravityNorm;

    /**
     * Contains 3x3 identify to be reused.
     */
    protected Matrix mIdentity;

    /**
     * Contains 3x3 temporary matrix.
     */
    protected Matrix mTmp1;

    /**
     * Contains 3x3 temporary matrix.
     */
    protected Matrix mTmp2;

    /**
     * Contains 3x1 temporary matrix.
     */
    protected Matrix mTmp3;

    /**
     * Contains 3x1 temporary matrix.
     */
    protected Matrix mTmp4;

    /**
     * Constructor.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final List<StandardDeviationBodyKinematics> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias. This must have length 3 and is expressed
     *             in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final double[] bias) {
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(final Matrix bias) {
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final Matrix bias, final Matrix initialMa) {
        this(bias);
        try {
            setInitialMa(initialMa);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position) {
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements) {
        this(measurements);
        mPosition = position;
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
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(measurements);
        mPosition = position;
        mListener = listener;
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
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        mPosition = position;
        mCommonAxisUsed = commonAxisUsed;
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
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        this(bias);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(position, measurements, bias);
        mListener = listener;
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(position, measurements, bias);
        mCommonAxisUsed = commonAxisUsed;
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
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        this(bias);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(position, measurements, bias);
        mListener = listener;
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
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(position, measurements, bias);
        mCommonAxisUsed = commonAxisUsed;
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
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(bias, initialMa);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(position, measurements, bias, initialMa);
        mListener = listener;
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(position, measurements, bias, initialMa);
        mCommonAxisUsed = commonAxisUsed;
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, bias, initialMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(final NEDPosition position) {
        mPosition = convertPosition(position);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements) {
        this(convertPosition(position), measurements);
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
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, listener);
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
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(convertPosition(position), measurements, commonAxisUsed);
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
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        this(convertPosition(position), measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, bias, listener);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(convertPosition(position), measurements, commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                bias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        this(convertPosition(position), measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, bias, listener);
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
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(convertPosition(position), measurements, commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, bias,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(convertPosition(position), measurements, bias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, bias, initialMa,
                listener);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(convertPosition(position), measurements, commonAxisUsed, bias,
                initialMa);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, bias,
                initialMa, listener);
    }

    /**
     * Gets x-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return x-coordinate of known accelerometer bias.
     */
    public double getBiasX() {
        return mBiasX;
    }

    /**
     * Sets x-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param biasX x-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasX(final double biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
    }

    /**
     * Gets y-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return y-coordinate of known accelerometer bias.
     */
    public double getBiasY() {
        return mBiasY;
    }

    /**
     * Sets y-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param biasY y-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasY(final double biasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = biasY;
    }

    /**
     * Gets z-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return z-coordinate of known accelerometer bias.
     */
    public double getBiasZ() {
        return mBiasZ;
    }

    /**
     * Sets z-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param biasZ z-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasZ(final double biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = biasZ;
    }

    /**
     * Gets x-coordinate of known accelerometer bias.
     *
     * @return x-coordinate of known accelerometer bias.
     */
    public Acceleration getBiasXAsAcceleration() {
        return new Acceleration(mBiasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets x-coordinate of known accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    public void getBiasXAsAcceleration(final Acceleration result) {
        result.setValue(mBiasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets x-coordinate of known accelerometer bias.
     *
     * @param biasX x-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasX(final Acceleration biasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = convertAcceleration(biasX);
    }

    /**
     * Gets y-coordinate of known accelerometer bias.
     *
     * @return y-coordinate of known accelerometer bias.
     */
    public Acceleration getBiasYAsAcceleration() {
        return new Acceleration(mBiasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets y-coordinate of known accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    public void getBiasYAsAcceleration(final Acceleration result) {
        result.setValue(mBiasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets y-coordinate of known accelerometer bias.
     *
     * @param biasY y-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasY(final Acceleration biasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = convertAcceleration(biasY);
    }

    /**
     * Gets z-coordinate of known accelerometer bias.
     *
     * @return z-coordinate of known accelerometer bias.
     */
    public Acceleration getBiasZAsAcceleration() {
        return new Acceleration(mBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets z-coordinate of known accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    public void getBiasZAsAcceleration(final Acceleration result) {
        result.setValue(mBiasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets z-coordinate of known accelerometer bias to be used to find a solution.
     *
     * @param biasZ z-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBiasZ(final Acceleration biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = convertAcceleration(biasZ);
    }

    /**
     * Sets known bias coordinates of accelerometer expressed in meters
     * per squared second (m/s^2).
     *
     * @param biasX x-coordinate of known accelerometer bias.
     * @param biasY y-coordinate of known accelerometer bias.
     * @param biasZ z-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(final double biasX, final double biasY,
                        final double biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
        mBiasY = biasY;
        mBiasZ = biasZ;
    }

    /**
     * Sets known bias coordinates of accelerometer.
     *
     * @param biasX x-coordinate of known accelerometer bias.
     * @param biasY y-coordinate of known accelerometer bias.
     * @param biasZ z-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(final Acceleration biasX,
                        final Acceleration biasY,
                        final Acceleration biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = convertAcceleration(biasX);
        mBiasY = convertAcceleration(biasY);
        mBiasZ = convertAcceleration(biasZ);
    }

    /**
     * Gets known accelerometer bias.
     *
     * @return known accelerometer bias.
     */
    public AccelerationTriad getBiasAsTriad() {
        return new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND,
                mBiasX, mBiasY, mBiasZ);
    }

    /**
     * Gets known accelerometer bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsTriad(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(mBiasX, mBiasY, mBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known accelerometer bias.
     *
     * @param bias accelerometer bias to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(final AccelerationTriad bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasX = convertAcceleration(bias.getValueX(), bias.getUnit());
        mBiasY = convertAcceleration(bias.getValueY(), bias.getUnit());
        mBiasZ = convertAcceleration(bias.getValueZ(), bias.getUnit());
    }

    /**
     * Gets initial x scaling factor.
     *
     * @return initial x scaling factor.
     */
    public double getInitialSx() {
        return mInitialSx;
    }

    /**
     * Sets initial x scaling factor.
     *
     * @param initialSx initial x scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialSx(final double initialSx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSx = initialSx;
    }

    /**
     * Gets initial y scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial y scaling factor.
     */
    public double getInitialSy() {
        return mInitialSy;
    }

    /**
     * Sets initial y scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSy initial y scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialSy(final double initialSy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSy = initialSy;
    }

    /**
     * Gets initial z scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial z scaling factor.
     */
    public double getInitialSz() {
        return mInitialSz;
    }

    /**
     * Sets initial z scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSz initial z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialSz(final double initialSz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSz = initialSz;
    }

    /**
     * Gets initial x-y cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x-y cross coupling error.
     */
    public double getInitialMxy() {
        return mInitialMxy;
    }

    /**
     * Sets initial x-y cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMxy initial x-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMxy(final double initialMxy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMxy = initialMxy;
    }

    /**
     * Gets initial x-z cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x-z cross coupling error.
     */
    public double getInitialMxz() {
        return mInitialMxz;
    }

    /**
     * Sets initial x-z cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMxz initial x-z cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMxz(final double initialMxz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMxz = initialMxz;
    }

    /**
     * Gets initial y-x cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial y-x cross coupling error.
     */
    public double getInitialMyx() {
        return mInitialMyx;
    }

    /**
     * Sets initial y-x cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMyx initial y-x cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMyx(final double initialMyx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMyx = initialMyx;
    }

    /**
     * Gets initial y-z cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial y-z cross coupling error.
     */
    public double getInitialMyz() {
        return mInitialMyz;
    }

    /**
     * Sets initial y-z cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMyz initial y-z cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMyz(final double initialMyz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMyz = initialMyz;
    }

    /**
     * Gets initial z-x cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial z-x cross coupling error.
     */
    public double getInitialMzx() {
        return mInitialMzx;
    }

    /**
     * Sets initial z-x cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMzx initial z-x cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMzx(final double initialMzx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMzx = initialMzx;
    }

    /**
     * Gets initial z-y cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial z-y cross coupling error.
     */
    public double getInitialMzy() {
        return mInitialMzy;
    }

    /**
     * Sets initial z-y cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMzy initial z-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialMzy(final double initialMzy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMzy = initialMzy;
    }

    /**
     * Sets initial scaling factors.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialScalingFactors(
            final double initialSx, final double initialSy, final double initialSz)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSx = initialSx;
        mInitialSy = initialSy;
        mInitialSz = initialSz;
    }

    /**
     * Sets initial cross coupling errors.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
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
     * Sets initial scaling factors and cross coupling errors.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
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
     * Gets known bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @return array containing coordinates of known bias.
     */
    public double[] getBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets known bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
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
     * Sets known bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param bias known bias to be set
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setBias(final double[] bias) throws LockedException {
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
     * Gets known bias as a column matrix.
     *
     * @return known bias as a column matrix.
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
     * Gets known bias as a column matrix.
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
     * Sets known bias as an array.
     *
     * @param bias bias to be set.
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
     * Gets initial scale factors and cross coupling errors matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial scale factors and cross coupling errors matrix.
     */
    public Matrix getInitialMa() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            getInitialMa(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets initial scale factors and cross coupling errors matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void getInitialMa(final Matrix result) {
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
     * Sets initial scale factors and cross coupling errors matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setInitialMa(final Matrix initialMa) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialMa.getRows() != BodyKinematics.COMPONENTS ||
                initialMa.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mInitialSx = initialMa.getElementAtIndex(0);
        mInitialMyx = initialMa.getElementAtIndex(1);
        mInitialMzx = initialMa.getElementAtIndex(2);

        mInitialMxy = initialMa.getElementAtIndex(3);
        mInitialSy = initialMa.getElementAtIndex(4);
        mInitialMzy = initialMa.getElementAtIndex(5);

        mInitialMxz = initialMa.getElementAtIndex(6);
        mInitialMyz = initialMa.getElementAtIndex(7);
        mInitialSz = initialMa.getElementAtIndex(8);
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
     * Gets a list of body kinematics measurements taken at a
     * given position with different unknown orientations and containing the
     * standard deviations of accelerometer and gyroscope measurements.
     *
     * @return list of body kinematics measurements.
     */
    public List<StandardDeviationBodyKinematics> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets a list of body kinematics measurements taken at a
     * given position with different unknown orientations and containing the
     * standard deviations of accelerometer and gyroscope measurements.
     *
     * @param measurements list of body kinematics measurements.
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
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public RobustKnownBiasAndPositionAccelerometerCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener)
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
        return mCommonAxisUsed ? MINIMUM_MEASUREMENTS_COMMON_Z_AXIS :
                MINIMUM_MEASUREMENTS_GENERAL;
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    public boolean isReady() {
        return mMeasurements != null && mMeasurements.size() >= getMinimumRequiredMeasurements()
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
     * Gets estimated accelerometer scale factors and ross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated accelerometer scale factors and cross coupling errors, or null
     * if not available.
     */
    public Matrix getEstimatedMa() {
        return mEstimatedMa;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    public Double getEstimatedSx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    public Double getEstimatedSy() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    public Double getEstimatedSz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    public Double getEstimatedMxy() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    public Double getEstimatedMxz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    public Double getEstimatedMyx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    public Double getEstimatedMyz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    public Double getEstimatedMzx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    public Double getEstimatedMzy() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(2, 1) : null;
    }

    /**
     * Gets estimated covariance matrix for estimated calibration solution.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy.
     * This is only available when result has been refined and covariance
     * is kept.
     *
     * @return estimated covariance matrix for estimated position.
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
     * This has to be at least {@link #getMinimumRequiredMeasurements()}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinimumRequiredMeasurements()}.
     */
    public void setPreliminarySubsetSize(
            final int preliminarySubsetSize) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < getMinimumRequiredMeasurements()) {
            throw new IllegalArgumentException();
        }

        mPreliminarySubsetSize = preliminarySubsetSize;
    }

    /**
     * Estimates accelerometer calibration parameters containing scale factors
     * and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    public abstract void calibrate() throws LockedException, NotReadyException,
            CalibrationException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param method robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator();
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator();
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator();
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator();
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        measurements);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        measurements);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        measurements);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        measurements);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias   known accelerometer bias. This must have length 3 and is expressed
     *               in meters per squared second (m/s^2).
     * @param method robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias   known accelerometer bias.
     * @param method robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final Matrix bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @param method    robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final Matrix bias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position position where body kinematics measures have been taken.
     * @param method   robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
        }
    }

    /**
     * Creates a robust accelerometer calibrator
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is
     *                     expressed in meters per squared second (m/s^2).
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is
     *                     expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and
     *                       is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position position where body kinematics measures have been taken.
     * @param method   robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is
     *                     expressed in meters per squared second (m/s^2).
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is
     *                     expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  list of body kinematics measurements taken at a given position with
     *                      different unknown orientations and containing the standard deviations
     *                      of accelerometer and gyroscope measurements.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  list of body kinematics measurements taken at a given position with
     *                      different unknown orientations and containing the standard deviations
     *                      of accelerometer and gyroscope measurements.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope. If true 7 samples are
     *                       required, otherwise 10.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than the minimum number of
     *                                  required samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope. If true 7 samples are
     *                       required, otherwise 10.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than the minimum number of
     *                                  required samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias. This must have length 3 and is
     *                      expressed in meters per squared second (m/s^2).
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias. This must have length 3 and is
     *                      expressed in meters per squared second (m/s^2).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or if
     *                                  quality scores array is smaller than 13
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or if
     *                                  quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias,
                        listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias.
     * @param initialMa     initial scale factors and cross coupling errors matrix.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias.
     * @param initialMa     initial scale factors and cross coupling errors matrix.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, initialMa,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, initialMa,
                        listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, initialMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, initialMa, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  list of body kinematics measurements taken at a given position with
     *                      different unknown orientations and containing the standard deviations
     *                      of accelerometer and gyroscope measurements.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  list of body kinematics measurements taken at a given position with
     *                      different unknown orientations and containing the standard deviations
     *                      of accelerometer and gyroscope measurements.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope. If true 7 samples are
     *                       required, otherwise 10.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than the minimum number of
     *                                  required samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope. If true 7 samples are
     *                       required, otherwise 10.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than the minimum number of
     *                                  required samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias. This must have length 3 and is
     *                      expressed in meters per squared second (m/s^2).
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias. This must have length 3 and is
     *                      expressed in meters per squared second (m/s^2).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or if
     *                                  quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or if
     *                                  quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias.
     * @param initialMa     initial scale factors and cross coupling errors matrix.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param bias          known accelerometer bias.
     * @param initialMa     initial scale factors and cross coupling errors matrix.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if quality scores array is smaller than 10
     *                                  samples.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, bias, initialMa, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias,
                        initialMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, bias,
                        initialMa, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, initialMa);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        bias, initialMa);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (7 or 10).
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        position, measurements, commonAxisUsed, bias, initialMa,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias,
                        initialMa, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed, bias,
                        initialMa, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final List<StandardDeviationBodyKinematics> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias known accelerometer bias. This must have length 3 and is expressed
     *             in meters per squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final double[] bias) {
        return create(bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final Matrix bias) {
        return create(bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final Matrix bias, final Matrix initialMa) {
        return create(bias, initialMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position position where body kinematics measures have been taken.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position) {
        return create(position, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements) {
        return create(position, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        return create(position, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is
     *                     expressed in meters per squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        return create(position, measurements, bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is
     *                     expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        return create(position, measurements, commonAxisUsed, bias,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, bias, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        return create(position, measurements, bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        return create(position, measurements, commonAxisUsed, bias,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, bias, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        return create(position, measurements, bias, initialMa,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, bias, initialMa, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        return create(position, measurements, commonAxisUsed, bias,
                initialMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, bias, initialMa,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position position where body kinematics measures have been taken.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position) {
        return create(position, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements) {
        return create(position, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        return create(position, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is
     *                     expressed in meters per squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        return create(position, measurements, bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is
     *                     expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, bias, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        return create(position, measurements, commonAxisUsed, bias,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is
     *                       expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, bias, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        return create(position, measurements, bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        return create(position, measurements, commonAxisUsed, bias,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, bias,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        return create(position, measurements, bias, initialMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, bias, initialMa, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        return create(position, measurements, commonAxisUsed, bias, initialMa,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownBiasAndPositionAccelerometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa,
            final RobustKnownBiasAndPositionAccelerometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, bias, initialMa,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Computes error of preliminary result respect a given measurement for current gravity norm.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final StandardDeviationBodyKinematics measurement,
            final PreliminaryResult preliminaryResult) {

        try {
            // We know that measured specific force is:
            // fmeas = ba + (I + Ma) * ftrue

            // fmeas - ba = (I + Ma) * ftrue

            // ftrue = (I + Ma)^-1 * (fmeas - ba)

            // We know that ||ftrue|| should be equal to the gravity value at current Earth
            // position
            // ||ftrue|| = g ~ 9.81 m/s^2

            final Matrix estimatedMa = preliminaryResult.mEstimatedMa;

            if (mIdentity == null) {
                mIdentity = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }

            if (mTmp1 == null) {
                mTmp1 = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }

            if (mTmp2 == null) {
                mTmp2 = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }

            if (mTmp3 == null) {
                mTmp3 = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            if (mTmp4 == null) {
                mTmp4 = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            mIdentity.add(estimatedMa, mTmp1);

            Utils.inverse(mTmp1, mTmp2);

            final BodyKinematics kinematics = measurement.getKinematics();
            final double fmeasX = kinematics.getFx();
            final double fmeasY = kinematics.getFy();
            final double fmeasZ = kinematics.getFz();

            mTmp3.setElementAtIndex(0, fmeasX - mBiasX);
            mTmp3.setElementAtIndex(1, fmeasY - mBiasY);
            mTmp3.setElementAtIndex(2, fmeasZ - mBiasZ);

            mTmp2.multiply(mTmp3, mTmp4);

            final double norm = Utils.normF(mTmp4);
            final double diff = mGravityNorm - norm;

            return diff * diff;

        } catch (final AlgebraException e) {
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
            result.mEstimatedMa = getInitialMa();

            mInnerCalibrator.setBias(mBiasX, mBiasY, mBiasZ);
            mInnerCalibrator.setInitialMa(result.mEstimatedMa);
            mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
            mInnerCalibrator.setPosition(mPosition);
            mInnerCalibrator.setMeasurements(measurements);
            mInnerCalibrator.calibrate();

            result.mEstimatedMa = mInnerCalibrator.getEstimatedMa();

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
                mInnerCalibrator.setBias(mBiasX, mBiasY, mBiasZ);
                mInnerCalibrator.setInitialMa(preliminaryResult.mEstimatedMa);
                mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mInnerCalibrator.setPosition(mPosition);
                mInnerCalibrator.setMeasurements(inlierMeasurements);
                mInnerCalibrator.calibrate();

                mEstimatedMa = mInnerCalibrator.getEstimatedMa();
                mEstimatedMse = mInnerCalibrator.getEstimatedMse();

                if (mKeepCovariance) {
                    mEstimatedCovariance = mInnerCalibrator.getEstimatedCovariance();
                } else {
                    mEstimatedCovariance = null;
                }
            } catch (final LockedException | CalibrationException | NotReadyException e) {
                mEstimatedCovariance = null;
                mEstimatedMa = preliminaryResult.mEstimatedMa;
                mEstimatedMse = 0.0;
            }
        } else {
            mEstimatedCovariance = null;
            mEstimatedMa = preliminaryResult.mEstimatedMa;
            mEstimatedMse = 0.0;
        }
    }

    /**
     * Computes gravity norm for current position.
     *
     * @return gravity norm for current position expressed in meters per squared second.
     */
    protected double computeGravityNorm() {
        final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                mPosition.getX(), mPosition.getY(), mPosition.getZ());
        return gravity.getNorm();
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
     * Converts acceleration value and unit to meters per squared second.
     *
     * @param value acceleration value.
     * @param unit  unit of acceleration value.
     * @return converted value.
     */
    private static double convertAcceleration(final double value, final AccelerationUnit unit) {
        return AccelerationConverter.convert(value, unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts acceleration instance to meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return convertAcceleration(acceleration.getValue().doubleValue(),
                acceleration.getUnit());
    }

    /**
     * Internal class containing estimated preliminary result.
     */
    protected static class PreliminaryResult {
        /**
         * Estimated accelerometer scale factors and cross coupling errors.
         * This is the product of matrix Ta containing cross coupling errors and Ka
         * containing scaling factors.
         * So tat:
         * <pre>
         *     Ma = [sx    mxy  mxz] = Ta*Ka
         *          [myx   sy   myz]
         *          [mzx   mzy  sz ]
         * </pre>
         * Where:
         * <pre>
         *     Ka = [sx 0   0 ]
         *          [0  sy  0 ]
         *          [0  0   sz]
         * </pre>
         * and
         * <pre>
         *     Ta = [1          -alphaXy    alphaXz ]
         *          [alphaYx    1           -alphaYz]
         *          [-alphaZx   alphaZy     1       ]
         * </pre>
         * Hence:
         * <pre>
         *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
         *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
         *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
         * </pre>
         * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
         * are considered to be zero if the accelerometer z-axis is assumed to be the same
         * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
         * becomes upper diagonal:
         * <pre>
         *     Ma = [sx    mxy  mxz]
         *          [0     sy   myz]
         *          [0     0    sz ]
         * </pre>
         * Values of this matrix are unitless.
         */
        private Matrix mEstimatedMa;
    }
}
