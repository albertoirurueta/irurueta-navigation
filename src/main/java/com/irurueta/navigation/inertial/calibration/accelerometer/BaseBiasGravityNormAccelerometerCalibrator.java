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
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.Collection;

/**
 * Abstract class to estimate accelerometer cross couplings and scaling factors when
 * accelerometer biases and gravity norm are known.
 * Gravity norm can be known because it has been directly provided or because position
 * respect Earth is known, and thus gravity norm can be computed.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared error
 * solution.
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
 *
 * @param <C> a calibrator type.
 * @param <L> a listener type.
 */
public abstract class BaseBiasGravityNormAccelerometerCalibrator<
        C extends BaseBiasGravityNormAccelerometerCalibrator<?, ?>,
        L extends BaseBiasGravityNormAccelerometerCalibratorListener<C>> {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS = 6;

    /**
     * Number of unknowns for the general case.
     */
    public static final int GENERAL_UNKNOWNS = 9;

    /**
     * Required minimum number of measurements when common z-axis is assumed.
     */
    public static final int MINIMUM_MEASUREMENTS_COMON_Z_AXIS = COMMON_Z_AXIS_UNKNOWNS + 1;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL = GENERAL_UNKNOWNS + 1;

    /**
     * Ground truth gravity norm to be expected at location where measurements have been made,
     * expressed in meters per squared second (m/s^2).
     */
    protected Double mGroundTruthGravityNorm;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter mFitter =
            new LevenbergMarquardtMultiDimensionFitter();

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
     * Contains a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     */
    private Collection<StandardDeviationBodyKinematics> mMeasurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private L mListener;

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
     * Estimated covariance matrix for estimated position.
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
     * Indicates whether estimator is running.
     */
    private boolean mRunning;

    /**
     * Internally holds x-coordinate of measured specific force during calibration.
     */
    private double mFmeasX;

    /**
     * Internally holds y-coordinate of measured specific force during calibration.
     */
    private double mFmeasY;

    /**
     * Internally holds z-coordinate of measured specific force during calibration.
     */
    private double mFmeasZ;

    /**
     * Internally holds measured specific force during calibration expressed as
     * a column matrix.
     */
    private Matrix mFmeas;

    /**
     * Internally holds cross-coupling errors during calibration.
     */
    private Matrix mM;

    /**
     * Internally holds inverse of cross-coupling errors during calibration.
     */
    private Matrix mInvM;

    /**
     * Internally holds biases during calibration in internal format.
     */
    private Matrix mB;

    /**
     * Internally holds computed true specific force during calibration.
     */
    private Matrix mFtrue;

    /**
     * Internally hold biases during calibration in external format.
     */
    private Matrix mBa;

    /**
     * Constructor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final L listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final L listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final L listener) {
        this(commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final L listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX x-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     * @param biasY y-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     * @param biasZ z-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ) {
        try {
            setBias(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    x-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasY    y-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasZ    z-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ,
            final L listener) {
        this(biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ) {
        this(biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final L listener) {
        this(measurements, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX x-coordinate of accelerometer bias.
     * @param biasY y-coordinate of accelerometer bias.
     * @param biasZ z-coordinate of accelerometer bias.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        try {
            setBias(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    x-coordinate of accelerometer bias.
     * @param biasY    y-coordinate of accelerometer bias.
     * @param biasZ    z-coordinate of accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final L listener) {
        this(biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final L listener) {
        this(measurements, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasY     y-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasZ     z-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(biasX, biasY, biasZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy,
                initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final L listener) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final L listener) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      x-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasY      y-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ);
        try {
            setInitialScalingFactorsAndCrossCouplingErrors(initialSx, initialSy, initialSz,
                    initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ);
        try {
            setInitialScalingFactorsAndCrossCouplingErrors(initialSx, initialSy, initialSz,
                    initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @param listener   listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias. This must have length 3 and is expressed
     *             in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
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
     * @param bias     known accelerometer bias. This must have length 3 and is expressed
     *                 in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final double[] bias, final L listener) {
        this(bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        this(bias);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias,
            final L listener) {
        this(measurements, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] bias) {
        this(bias);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] bias,
            final L listener) {
        this(commonAxisUsed, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(commonAxisUsed, bias);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
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
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final L listener) {
        this(measurements, commonAxisUsed, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(final Matrix bias) {
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Matrix bias,
            final L listener) {
        this(bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        this(bias);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final L listener) {
        this(measurements, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias) {
        this(bias);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final L listener) {
        this(commonAxisUsed, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(commonAxisUsed, bias);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final L listener) {
        this(measurements, commonAxisUsed, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
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
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Matrix bias, final Matrix initialMa,
            final L listener) {
        this(bias, initialMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(bias, initialMa);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final L listener) {
        this(measurements, bias, initialMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(bias, initialMa);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa, final L listener) {
        this(commonAxisUsed, bias, initialMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
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
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(commonAxisUsed, bias, initialMa);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
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
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa, final L listener) {
        this(measurements, commonAxisUsed, bias, initialMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm) {
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final L listener) {
        this(groundTruthGravityNorm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements) {
        this(groundTruthGravityNorm);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final L listener) {
        this(measurements, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.     *
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final L listener) {
        this(commonAxisUsed, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final L listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ) {
        this(biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ, final L listener) {
        this(biasX, biasY, biasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ) {
        this(measurements, biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ,
            final L listener) {
        this(measurements, biasX, biasY, biasZ,
                listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ) {
        this(measurements, commonAxisUsed, biasX, biasY,
                biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ,
            final L listener) {
        this(measurements, commonAxisUsed,
                biasX, biasY, biasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final L listener) {
        this(biasX, biasY, biasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        this(measurements, biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final L listener) {
        this(measurements, biasX, biasY, biasZ,
                listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(measurements, commonAxisUsed,
                biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy,
                initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final L listener) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX,
            final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final L listener) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final L listener) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final L listener) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final L listener) {
        this(biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double[] bias) {
        this(bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double[] bias,
            final L listener) {
        this(bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        this(measurements, bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias, final L listener) {
        this(measurements, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double[] bias) {
        this(commonAxisUsed, bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double[] bias, final L listener) {
        this(commonAxisUsed, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(measurements, commonAxisUsed, bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias, final L listener) {
        this(measurements, commonAxisUsed, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix bias) {
        this(bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix bias, final L listener) {
        this(bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        this(measurements, bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final L listener) {
        this(measurements, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias) {
        this(commonAxisUsed, bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final L listener) {
        this(commonAxisUsed, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(measurements, commonAxisUsed, bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final L listener) {
        this(measurements, commonAxisUsed, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix bias, final Matrix initialMa) {
        this(bias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Matrix bias, final Matrix initialMa,
            final L listener) {
        this(bias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(measurements, bias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final L listener) {
        this(measurements, bias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(commonAxisUsed, bias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa, final L listener) {
        this(commonAxisUsed, bias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(measurements, commonAxisUsed, bias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa, final L listener) {
        this(measurements, commonAxisUsed, bias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Gets ground truth gravity norm to be expected at location where measurements have been made,
     * expressed in meter per squared second (m/s^2).
     *
     * @return ground truth gravity norm or null.
     */
    public Double getGroundTruthGravityNorm() {
        return mGroundTruthGravityNorm;
    }

    /**
     * Gets ground truth gravity norm to be expected at location where measurements have been made.
     *
     * @return ground truth gravity norm or null.
     */
    public Acceleration getGroundTruthGravityNormAsAcceleration() {
        return mGroundTruthGravityNorm != null
                ? new Acceleration(mGroundTruthGravityNorm, AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets ground truth gravity norm to be expected at location where measurements have been made.
     *
     * @param result instance where result will be stored.
     * @return true if ground truth gravity norm has been defined, false if it is not available yet.
     */
    public boolean getGroundTruthGravityNormAsAcceleration(final Acceleration result) {
        if (mGroundTruthGravityNorm != null) {
            result.setValue(mGroundTruthGravityNorm);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
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
     *
     * @return initial y scaling factor.
     */
    public double getInitialSy() {
        return mInitialSy;
    }

    /**
     * Sets initial y scaling factor.
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
     *
     * @return initial z scaling factor.
     */
    public double getInitialSz() {
        return mInitialSz;
    }

    /**
     * Sets initial z scaling factor.
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
     *
     * @return initial x-y cross coupling error.
     */
    public double getInitialMxy() {
        return mInitialMxy;
    }

    /**
     * Sets initial x-y cross coupling error.
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
     *
     * @return initial x-z cross coupling error.
     */
    public double getInitialMxz() {
        return mInitialMxz;
    }

    /**
     * Sets initial x-z cross coupling error.
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
     *
     * @return initial y-x cross coupling error.
     */
    public double getInitialMyx() {
        return mInitialMyx;
    }

    /**
     * Sets initial y-x cross coupling error.
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
     *
     * @return initial y-z cross coupling error.
     */
    public double getInitialMyz() {
        return mInitialMyz;
    }

    /**
     * Sets initial y-z cross coupling error.
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
     *
     * @return initial z-x cross coupling error.
     */
    public double getInitialMzx() {
        return mInitialMzx;
    }

    /**
     * Sets initial z-x cross coupling error.
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
     *
     * @return initial z-y cross coupling error.
     */
    public double getInitialMzy() {
        return mInitialMzy;
    }

    /**
     * Sets initial z-y cross coupling error.
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
     * Gets a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     *
     * @return collection of body kinematics measurements at a known position
     * with unknown orientations.
     */
    public Collection<StandardDeviationBodyKinematics> getMeasurements() {
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
            final Collection<StandardDeviationBodyKinematics> measurements)
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
    public L getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final L listener) throws LockedException {
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
        return mCommonAxisUsed ? MINIMUM_MEASUREMENTS_COMON_Z_AXIS :
                MINIMUM_MEASUREMENTS_GENERAL;
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    public boolean isReady() {
        return mMeasurements != null && mMeasurements.size() >= getMinimumRequiredMeasurements()
                && mGroundTruthGravityNorm != null;
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
     * Estimates accelerometer calibration parameters containing scale factors
     * and cross-coupling errors.
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
                //noinspection unchecked
                mListener.onCalibrateStart((C) this);
            }

            if (mCommonAxisUsed) {
                calibrateCommonAxis();
            } else {
                calibrateGeneral();
            }

            if (mListener != null) {
                //noinspection unchecked
                mListener.onCalibrateEnd((C) this);
            }

        } catch (final AlgebraException | FittingException
                | com.irurueta.numerical.NotReadyException e) {
            throw new CalibrationException(e);
        } finally {
            mRunning = false;
        }
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
     * Gets estimated covariance matrix for estimated calibration parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy.
     *
     * @return estimated covariance matrix for estimated calibration parameters.
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
     * Gets estimated mean square error respect to provided measurements.
     *
     * @return estimated mean square error respect to provided measurements.
     */
    public double getEstimatedMse() {
        return mEstimatedMse;
    }

    /**
     * Converts acceleration instance to meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value.
     */
    protected static double convertAcceleration(final Acceleration acceleration) {
        return convertAcceleration(acceleration.getValue().doubleValue(),
                acceleration.getUnit());
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
     * Internally sets ground truth gravity norm to be expected at location where
     * measurements have been made, expressed in meters per squared second
     * (m/s^2).
     *
     * @param groundTruthGravityNorm ground truth gravity norm or null if
     *                               undefined.
     * @throws IllegalArgumentException if provided value is negative.
     */
    protected void internalSetGroundTruthGravityNorm(final Double groundTruthGravityNorm) {
        if (groundTruthGravityNorm != null && groundTruthGravityNorm < 0.0) {
            throw new IllegalArgumentException();
        }
        mGroundTruthGravityNorm = groundTruthGravityNorm;
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     */
    private void setInputData() throws WrongSizeException {

        final double g = mGroundTruthGravityNorm;
        final double g2 = g * g;

        final int numMeasurements = mMeasurements.size();
        final Matrix x = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final double[] y = new double[numMeasurements];
        final double[] specificForceStandardDeviations = new double[numMeasurements];
        int i = 0;
        for (final StandardDeviationBodyKinematics measurement : mMeasurements) {
            final BodyKinematics measuredKinematics = measurement.getKinematics();

            final double fmeasX = measuredKinematics.getFx();
            final double fmeasY = measuredKinematics.getFy();
            final double fmeasZ = measuredKinematics.getFz();

            x.setElementAt(i, 0, fmeasX);
            x.setElementAt(i, 1, fmeasY);
            x.setElementAt(i, 2, fmeasZ);

            y[i] = g2;

            specificForceStandardDeviations[i] =
                    measurement.getSpecificForceStandardDeviation();

            i++;
        }

        mFitter.setInputData(x, y, specificForceStandardDeviations);
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws FittingException                         if Levenberg-Marquardt fails for numerical reasons.
     * @throws AlgebraException                         if there are numerical instabilities that prevent
     *                                                  matrix inversion.
     * @throws com.irurueta.numerical.NotReadyException never happens.
     */
    private void calibrateGeneral() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // accelerometer model can be better expressed as:
        // fmeas = T*K*(ftrue + b)
        // fmeas = M*(ftrue + b)
        // fmeas = M*ftrue + M*b

        //where:
        // M = I + Ma
        // ba = M*b = (I + Ma)*b --> b = M^-1*ba

        // We know that the norm of the true specific force is equal to the amount
        // of gravity at a certain Earth position
        // ||ftrue|| = ||g|| ~ 9.81 m/s^2

        // Hence:
        // fmeas - M*b = M*ftrue

        // M^-1 * (fmeas - M*b) = ftrue

        // ||g||^2 = ||ftrue||^2 = (M^-1 * (fmeas - M*b))^T * (M^-1 * (fmeas - M*b))
        // ||g||^2 = (fmeas - M*b)^T*(M^-1)^T * M^-1 * (fmeas - M*b)
        // ||g||^2 = (fmeas - M * b)^T * ||M^-1||^2 * (fmeas - M * b)
        // ||g||^2 = ||fmeas - M * b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [m21 	m22 	m23]
        //     [m31 	m32 	m33]

        // Notice that bias b is known, hence only terms in matrix M need to be estimated

        final GradientEstimator gradientEstimator = new GradientEstimator(
                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] point) throws EvaluationException {
                        return evaluateGeneral(point);
                    }
                });

        final Matrix initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMa());

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Input points are measured specific force coordinates
                        return BodyKinematics.COMPONENTS;
                    }

                    @Override
                    public double[] createInitialParametersArray() {
                        // cross coupling errors M
                        return initialM.toArray();
                    }

                    @Override
                    public double evaluate(final int i, final double[] point,
                                           final double[] params, final double[] derivatives)
                            throws EvaluationException {

                        mFmeasX = point[0];
                        mFmeasY = point[1];
                        mFmeasZ = point[2];

                        gradientEstimator.gradient(params, derivatives);

                        return evaluateGeneral(params);
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
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws FittingException                         if Levenberg-Marquardt fails for numerical reasons.
     * @throws AlgebraException                         if there are numerical instabilities that prevent
     *                                                  matrix inversion.
     * @throws com.irurueta.numerical.NotReadyException never happens.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // accelerometer model can be better expressed as:
        // fmeas = T*K*(ftrue + b)
        // fmeas = M*(ftrue + b)
        // fmeas = M*ftrue + M*b

        //where:
        // M = I + Ma
        // ba = M*b = (I + Ma)*b --> b = M^-1*ba

        // We know that the norm of the true specific force is equal to the amount
        // of gravity at a certain Earth position
        // ||ftrue|| = ||g|| ~ 9.81 m/s^2

        // Hence:
        // fmeas - M*b = M*ftrue

        // M^-1 * (fmeas - M*b) = ftrue

        // ||g||^2 = ||ftrue||^2 = (M^-1 * (fmeas - M*b))^T * (M^-1 * (fmeas - M*b))
        // ||g||^2 = (fmeas - M*b)^T*(M^-1)^T * M^-1 * (fmeas - M*b)
        // ||g||^2 = (fmeas - M * b)^T * ||M^-1||^2 * (fmeas - M * b)
        // ||g||^2 = ||fmeas - M * b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [0 		m22 	m23]
        //     [0 	 	0 		m33]

        // Notice that bias b is known, hence only terms in matrix M need to be estimated

        final GradientEstimator gradientEstimator = new GradientEstimator(
                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double[] point) throws EvaluationException {
                        return evaluateCommonAxis(point);
                    }
                });

        final Matrix initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMa());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Input points are measured specific force coordinates
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
                    public double evaluate(final int i, final double[] point,
                                           final double[] params, final double[] derivatives)
                            throws EvaluationException {

                        mFmeasX = point[0];
                        mFmeasY = point[1];
                        mFmeasZ = point[2];

                        gradientEstimator.gradient(params, derivatives);

                        return evaluateCommonAxis(params);
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

        // taking into account that:
        // Ma = [sx  mxy  mxz] = [m11  m12  m13]
        //      [myx sy   myz]   [m21  m22  m23]
        //      [mzx mzy  sz ]   [m31  m32  m33]

        // propagate covariance so that all parameters are taken into account
        // in the order: sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy

        // We define a lineal function mapping original parameters for the
        // common axis case to the general case
        //[sx'] = [1  0  0  0  0  0][sx ]
        //[sy']   [0  0  1  0  0  0][mxy]
        //[sz']   [0  0  0  0  0  1][sy ]
        //[mxy']  [0  1  0  0  0  0][mxz]
        //[mxz']  [0  0  0  1  0  0][myz]
        //[myx']  [0  0  0  0  0  0][sz ]
        //[myz']  [0  0  0  0  1  0]
        //[mzx']  [0  0  0  0  0  0]
        //[mzy']  [0  0  0  0  0  0]

        // As defined in com.irurueta.statistics.MultivariateNormalDist,
        // if we consider the jacobian of the lineal application the matrix shown
        // above, then covariance can be propagated as follows
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);
        jacobian.setElementAt(0, 0, 1.0);
        jacobian.setElementAt(1, 2, 1.0);
        jacobian.setElementAt(2, 5, 1.0);
        jacobian.setElementAt(3, 1, 1.0);
        jacobian.setElementAt(4, 3, 1.0);
        jacobian.setElementAt(6, 4, 1.0);
        // propagated covariance is J * Cov * J'
        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
    }

    /**
     * Makes proper conversion of internal cross-coupling and bias matrices.
     *
     * @param m internal cross-coupling matrix.
     */
    private void setResult(final Matrix m) {
        // Because:
        // M = I + Ma

        // Then:
        // Ma = M - I

        if (mEstimatedMa == null) {
            mEstimatedMa = m;
        } else {
            mEstimatedMa.copyFrom(m);
        }

        for (int i = 0; i < BodyKinematics.COMPONENTS; i++) {
            mEstimatedMa.setElementAt(i, i,
                    mEstimatedMa.getElementAt(i, i) - 1.0);
        }

        // since only a constant term is subtracted, covariance is preserved
        mEstimatedCovariance = mFitter.getCovar();
        mEstimatedChiSq = mFitter.getChisq();
        mEstimatedMse = mFitter.getMse();
    }

    /**
     * Computes estimated true specific force squared norm using current measured
     * specific force and provided parameters for the general case.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the general purpose case.
     *               Must have length 9.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateGeneral(final double[] params) throws EvaluationException {
        final double m11 = params[0];
        final double m21 = params[1];
        final double m31 = params[2];

        final double m12 = params[3];
        final double m22 = params[4];
        final double m32 = params[5];

        final double m13 = params[6];
        final double m23 = params[7];
        final double m33 = params[8];

        return evaluate(m11, m21, m31, m12, m22, m32,
                m13, m23, m33);
    }

    /**
     * Computes estimated true specific force squared norm using current measured
     * specific force and provided parameters when common z-axis is assumed.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the common z-axis case.
     *               Must have length 6.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateCommonAxis(final double[] params) throws EvaluationException {
        final double m11 = params[0];

        final double m12 = params[1];
        final double m22 = params[2];

        final double m13 = params[3];
        final double m23 = params[4];
        final double m33 = params[5];

        return evaluate(m11, 0.0, 0.0, m12, m22, 0.0,
                m13, m23, m33);
    }

    /**
     * Computes estimated true specific force squared norm using current measured
     * specific force and provided parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param m11 element 1,1 of cross-coupling error matrix.
     * @param m21 element 2,1 of cross-coupling error matrix.
     * @param m31 element 3,1 of cross-coupling error matrix.
     * @param m12 element 1,2 of cross-coupling error matrix.
     * @param m22 element 2,2 of cross-coupling error matrix.
     * @param m32 element 3,2 of cross-coupling error matrix.
     * @param m13 element 1,3 of cross-coupling error matrix.
     * @param m23 element 2,3 of cross-coupling error matrix.
     * @param m33 element 3,3 of cross-coupling error matrix.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluate(final double m11, final double m21, final double m31,
                            final double m12, final double m22, final double m32,
                            final double m13, final double m23, final double m33)
            throws EvaluationException {

        // fmeas = M*(ftrue + b)

        // ftrue = M^-1*fmeas - b

        try {
            if (mFmeas == null) {
                mFmeas = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (mM == null) {
                mM = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }
            if (mInvM == null) {
                mInvM = new Matrix(BodyKinematics.COMPONENTS,
                        BodyKinematics.COMPONENTS);
            }
            if (mB == null) {
                mB = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (mFtrue == null) {
                mFtrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (mBa == null) {
                mBa = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            getBiasAsMatrix(mBa);

            mFmeas.setElementAtIndex(0, mFmeasX);
            mFmeas.setElementAtIndex(1, mFmeasY);
            mFmeas.setElementAtIndex(2, mFmeasZ);

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

            // b = M^-1*ba
            mInvM.multiply(mBa, mB);

            mInvM.multiply(mFmeas, mFtrue);
            mFtrue.subtract(mB);

            final double norm = Utils.normF(mFtrue);
            return norm * norm;

        } catch (final AlgebraException e) {
            throw new EvaluationException(e);
        }
    }
}
