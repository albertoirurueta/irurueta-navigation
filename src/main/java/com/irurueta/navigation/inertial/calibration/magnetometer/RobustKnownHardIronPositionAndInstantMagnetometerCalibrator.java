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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.io.IOException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.List;

/**
 * This is an abstract class to robustly estimate magnetometer cross
 * couplings and scaling factors.
 * <p>
 * To use this calibrator at least 7 measurements taken at a single known
 * position and instant must be taken at 7 different unknown orientations and
 * zero velocity when common z-axis is assumed, otherwise at least 10
 * measurements are required.
 * <p>
 * Measured magnetic flux density is assumed to follow the model shown below:
 * <pre>
 *     mBmeas = bm + (I + Mm) * mBtrue + w
 * </pre>
 * Where:
 * - mBmeas is the measured magnetic flux density. This is a 3x1 vector.
 * - bm is magnetometer hard-iron bias. Ideally, on a perfect magnetometer,
 * this should be a 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Mm is the 3x3 soft-iron matrix containing cross-couplings and scaling
 * factors. Ideally, on a perfect magnetometer, this should be a 3x3 zero
 * matrix.
 * - mBtrue is ground-truth magnetic flux density. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 * Notice that this calibrator assumes that all measurements are taken in
 * a short span of time where Earth magnetic field can be assumed to be
 * constant at provided location and instant.
 */
public abstract class RobustKnownHardIronPositionAndInstantMagnetometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements when common z-axis is assumed.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            KnownHardIronPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL =
            KnownHardIronPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;

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
     * Contains a list of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     */
    protected List<StandardDeviationBodyMagneticFluxDensity> mMeasurements;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener mListener;

    /**
     * Indicates whether estimator is running.
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
     * Data related to inlier found after calibration.
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
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from soft-iron (Mm) matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Estimated magnetometer soft-iron matrix containing scale factors
     * and cross coupling errors.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     */
    private Matrix mEstimatedMm;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean mKeepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Estimated covariance matrix for estimated parameters.
     */
    private Matrix mEstimatedCovariance;

    /**
     * Estimated mean square error respect to provided measurements.
     */
    private double mEstimatedMse;

    /**
     * Known x-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mHardIronX;

    /**
     * Known y-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mHardIronY;

    /**
     * Known z-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mHardIronZ;

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
     * Position where body magnetic flux density measurements have been
     * taken.
     */
    private NEDPosition mPosition;

    /**
     * Timestamp expressed as decimal year where magnetic flux density
     * measurements have been measured.
     */
    private Double mYear = convertTime(System.currentTimeMillis());

    /**
     * Contains Earth's magnetic model.
     */
    private WorldMagneticModel mMagneticModel;

    /**
     * Inner calibrator to compute calibration for each subset of data or during
     * final refining.
     */
    private final KnownHardIronPositionAndInstantMagnetometerCalibrator mInnerCalibrator =
            new KnownHardIronPositionAndInstantMagnetometerCalibrator();

    /**
     * Contains magnetic field norm for current position to be reused
     * during calibration.
     */
    protected double mMagneticDensityNorm;

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
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final WorldMagneticModel magneticModel) {
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param hardIron known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] hardIron) {
        try {
            setHardIron(hardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param hardIron known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final Matrix hardIron) {
        try {
            setHardIron(hardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param hardIron  known hard-iron.
     * @param initialMm initial soft-iron matrix containing scale factors
     *                  and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final Matrix hardIron, final Matrix initialMm) {
        this(hardIron);
        try {
            setInitialMm(initialMm);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position) {
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        this(position);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        this(position, measurements);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron) {
        this(hardIron);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron) {
        this(position, measurements, hardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron) {
        this(hardIron);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron) {
        this(position, measurements, hardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm) {
        this(hardIron, initialMm);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, hardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm) {
        this(position, measurements, hardIron, initialMm);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, hardIron,
                initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position) {
        this(convertPosition(position));
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        this(convertPosition(position), measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        this(convertPosition(position), measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron) {
        this(convertPosition(position), measurements,
                hardIron);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, hardIron,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron) {
        this(convertPosition(position), measurements, commonAxisUsed,
                hardIron);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron) {
        this(convertPosition(position), measurements, hardIron);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, hardIron,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron) {
        this(convertPosition(position), measurements, commonAxisUsed,
                hardIron);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm) {
        this(convertPosition(position), measurements, hardIron,
                initialMm);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, hardIron,
                initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm) {
        this(convertPosition(position), measurements, commonAxisUsed,
                hardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                hardIron, initialMm, listener);
    }

    /**
     * Gets known x-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return known x-coordinate of magnetometer hard-iron bias.
     */
    public double getHardIronX() {
        return mHardIronX;
    }

    /**
     * Sets known x-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronX known x-coordinate of magnetometer
     *                  hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIronX(final double hardIronX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronX = hardIronX;
    }

    /**
     * Gets known y-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return known y-coordinate of magnetometer hard-iron bias.
     */
    public double getHardIronY() {
        return mHardIronY;
    }

    /**
     * Sets known y-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronY known y-coordinate of magnetometer
     *                  hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIronY(final double hardIronY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronY = hardIronY;
    }

    /**
     * Gets known z-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return known z-coordinate of magnetometer hard-iron bias.
     */
    public double getHardIronZ() {
        return mHardIronZ;
    }

    /**
     * Sets known z-coordinate of magnetometer hard-iron bias.
     * This is expressed in meters Teslas (T).
     *
     * @param hardIronZ known z-coordinate of magnetometer
     *                  hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIronZ(final double hardIronZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronZ = hardIronZ;
    }

    /**
     * Gets known x coordinate of magnetometer hard-iron.
     *
     * @return x coordinate of magnetometer hard-iron.
     */
    public MagneticFluxDensity getHardIronXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mHardIronX,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known x coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    public void getHardIronXAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(mHardIronX);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets known x-coordinate of magnetometer hard-iron.
     *
     * @param hardIronX known x-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIronX(final MagneticFluxDensity hardIronX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronX = convertMagneticFluxDensity(hardIronX);
    }

    /**
     * Gets known y coordinate of magnetometer hard-iron.
     *
     * @return y coordinate of magnetometer hard-iron.
     */
    public MagneticFluxDensity getHardIronYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mHardIronY,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known y coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    public void getHardIronYAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(mHardIronY);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets known y-coordinate of magnetometer hard-iron.
     *
     * @param hardIronY known y-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIronY(final MagneticFluxDensity hardIronY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronY = convertMagneticFluxDensity(hardIronY);
    }

    /**
     * Gets known z coordinate of magnetometer hard-iron.
     *
     * @return z coordinate of magnetometer hard-iron.
     */
    public MagneticFluxDensity getHardIronZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mHardIronZ,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known z coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    public void getHardIronZAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(mHardIronZ);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets known z-coordinate of magnetometer hard-iron.
     *
     * @param hardIronZ known z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIronZ(final MagneticFluxDensity hardIronZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronZ = convertMagneticFluxDensity(hardIronZ);
    }

    /**
     * Sets known hard-iron bias coordinates of magnetometer expressed in
     * Teslas (T).
     *
     * @param hardIronX x-coordinate of magnetometer
     *                  known hard-iron bias.
     * @param hardIronY y-coordinate of magnetometer
     *                  known hard-iron bias.
     * @param hardIronZ z-coordinate of magnetometer
     *                  known hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIron(
            final double hardIronX,
            final double hardIronY,
            final double hardIronZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronX = hardIronX;
        mHardIronY = hardIronY;
        mHardIronZ = hardIronZ;
    }

    /**
     * Sets known hard-iron coordinates.
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron.
     * @param hardIronY y-coordinate of magnetometer hard-iron.
     * @param hardIronZ z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIron(
            final MagneticFluxDensity hardIronX,
            final MagneticFluxDensity hardIronY,
            final MagneticFluxDensity hardIronZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronX = convertMagneticFluxDensity(hardIronX);
        mHardIronY = convertMagneticFluxDensity(hardIronY);
        mHardIronZ = convertMagneticFluxDensity(hardIronZ);
    }

    /**
     * Gets known hard-iron.
     *
     * @return known hard-iron.
     */
    public MagneticFluxDensityTriad getHardIronAsTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                mHardIronX, mHardIronY, mHardIronZ);
    }

    /**
     * Gets known hard-iron.
     *
     * @param result instance where result will be stored.
     */
    public void getHardIronAsTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(
                mHardIronX, mHardIronY, mHardIronZ,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets known hard-iron.
     *
     * @param hardIron hard-iron to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setHardIron(final MagneticFluxDensityTriad hardIron)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mHardIronX = convertMagneticFluxDensity(hardIron.getValueX(), hardIron.getUnit());
        mHardIronY = convertMagneticFluxDensity(hardIron.getValueY(), hardIron.getUnit());
        mHardIronZ = convertMagneticFluxDensity(hardIron.getValueZ(), hardIron.getUnit());
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
    public void setInitialSx(final double initialSx)
            throws LockedException {
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
    public void setInitialSy(final double initialSy)
            throws LockedException {
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
    public void setInitialSz(final double initialSz)
            throws LockedException {
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
    public void setInitialMxy(final double initialMxy)
            throws LockedException {
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
    public void setInitialMxz(final double initialMxz)
            throws LockedException {
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
    public void setInitialMyx(final double initialMyx)
            throws LockedException {
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
    public void setInitialMyz(final double initialMyz)
            throws LockedException {
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
    public void setInitialMzx(final double initialMzx)
            throws LockedException {
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
    public void setInitialMzy(final double initialMzy)
            throws LockedException {
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
            final double initialSx,
            final double initialSy,
            final double initialSz)
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
            final double initialMxy,
            final double initialMxz,
            final double initialMyx,
            final double initialMyz,
            final double initialMzx,
            final double initialMzy)
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
            final double initialSx,
            final double initialSy,
            final double initialSz,
            final double initialMxy,
            final double initialMxz,
            final double initialMyx,
            final double initialMyz,
            final double initialMzx,
            final double initialMzy)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        setInitialScalingFactors(initialSx, initialSy, initialSz);
        setInitialCrossCouplingErrors(initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
    }

    /**
     * Gets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @return array containing coordinates of known bias.
     */
    public double[] getHardIron() {
        final double[] result = new double[
                BodyMagneticFluxDensity.COMPONENTS];
        getHardIron(result);
        return result;
    }

    /**
     * Gets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void getHardIron(final double[] result) {
        if (result.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = mHardIronX;
        result[1] = mHardIronY;
        result[2] = mHardIronZ;
    }

    /**
     * Sets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param hardIron known hard-iron.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setHardIron(final double[] hardIron)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (hardIron.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        mHardIronX = hardIron[0];
        mHardIronY = hardIron[1];
        mHardIronZ = hardIron[2];
    }

    /**
     * Gets known hard-iron bias as a column matrix.
     *
     * @return known hard-iron bias as a column matrix.
     */
    public Matrix getHardIronAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    1);
            getHardIronAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known hard-iron bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void getHardIronAsMatrix(final Matrix result) {
        if (result.getRows() != BodyMagneticFluxDensity.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mHardIronX);
        result.setElementAtIndex(1, mHardIronY);
        result.setElementAtIndex(2, mHardIronZ);
    }

    /**
     * Sets known hard-iron bias as a column matrix.
     *
     * @param hardIron known hard-iron bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setHardIron(final Matrix hardIron)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (hardIron.getRows() != BodyMagneticFluxDensity.COMPONENTS
                || hardIron.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mHardIronX = hardIron.getElementAtIndex(0);
        mHardIronY = hardIron.getElementAtIndex(1);
        mHardIronZ = hardIron.getElementAtIndex(2);
    }

    /**
     * Gets initial scale factors and cross coupling errors matrix.
     *
     * @return initial scale factors and cross coupling errors matrix.
     */
    public Matrix getInitialMm() {
        Matrix result;
        try {
            result = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
            getInitialMm(result);
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
    public void getInitialMm(final Matrix result) {
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
     * @param initialMm initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setInitialMm(final Matrix initialMm)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialMm.getRows() != BodyKinematics.COMPONENTS ||
                initialMm.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mInitialSx = initialMm.getElementAtIndex(0);
        mInitialMyx = initialMm.getElementAtIndex(1);
        mInitialMzx = initialMm.getElementAtIndex(2);

        mInitialMxy = initialMm.getElementAtIndex(3);
        mInitialSy = initialMm.getElementAtIndex(4);
        mInitialMzy = initialMm.getElementAtIndex(5);

        mInitialMxz = initialMm.getElementAtIndex(6);
        mInitialMyz = initialMm.getElementAtIndex(7);
        mInitialSz = initialMm.getElementAtIndex(8);
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken.
     *
     * @return position where body magnetic flux density measurements have
     * been taken.
     */
    public NEDPosition getNedPosition() {
        return mPosition;
    }

    /**
     * Sets position where body magnetic flux density measurements have been
     * taken.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final NEDPosition position)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mPosition = position;
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @return position where body magnetic flux density measurements have
     * been taken or null if not available.
     */
    public ECEFPosition getEcefPosition() {
        if (mPosition != null) {
            final ECEFPosition result = new ECEFPosition();
            getEcefPosition(result);
            return result;
        } else {
            return null;
        }
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if ECEF position could be computed, false otherwise.
     */
    public boolean getEcefPosition(final ECEFPosition result) {

        if (mPosition != null) {
            final ECEFVelocity velocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                    mPosition.getLatitude(), mPosition.getLongitude(),
                    mPosition.getHeight(), 0.0, 0.0, 0.0,
                    result, velocity);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @param position position where body magnetic flux density have been
     *                 taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final ECEFPosition position)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mPosition = convertPosition(position);
    }

    /**
     * Gets timestamp expressed as decimal year where magnetic flux density
     * measurements have been measured.
     *
     * @return timestamp expressed as decimal year or null if not defined.
     */
    public Double getYear() {
        return mYear;
    }

    /**
     * Sets timestamp expressed as decimal year where magnetic flux density
     * measurements have been measured.
     *
     * @param year timestamp expressed as decimal year.
     * @throws LockedException if calibrator is currently running.
     */
    public void setYear(final Double year) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mYear = year;
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param timestampMillis a timestamp expressed in milliseocnds since
     *                        epoch time (January 1st, 1970 at midnight).
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final Long timestampMillis) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mYear = convertTime(timestampMillis);
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param date a date instance containing a timestamp.
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final Date date) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mYear = convertTime(date);
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param calendar a calendar instance containing a timestamp.
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final GregorianCalendar calendar)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mYear = convertTime(calendar);
    }

    /**
     * Gets collection of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     *
     * @return collection of body magnetic flux density measurements at
     * a known position and timestamp with unknown orientations.
     */
    public List<StandardDeviationBodyMagneticFluxDensity> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets collection of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements at a known position and timestamp
     *                     with unknown orientations.
     * @throws LockedException if calibrator is currently running.
     */
    public void setMeasurements(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMeasurements = measurements;
    }

    /**
     * Indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from Mm (soft-iron) matrix.
     *
     * @return true if z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer, false otherwise.
     */
    public boolean isCommonAxisUsed() {
        return mCommonAxisUsed;
    }

    /**
     * Specifies whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Mm matrix.
     *
     * @param commonAxisUsed true if z-axis is assumed to be common for
     *                       accelerometer, gyroscope and magnetometer, false
     *                       otherwise.
     * @throws LockedException if estimator is currently running.
     */
    public void setCommonAxisUsed(final boolean commonAxisUsed)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Gets listener to handle events raised by this calibrator.
     *
     * @return listener to handle events raised by this calibrator.
     */
    public RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener)
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
     * Indicates whether calibrator is ready to start the estimator.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    public boolean isReady() {
        return mMeasurements != null
                && mMeasurements.size() >= getMinimumRequiredMeasurements()
                && mPosition != null && mYear != null;
    }

    /**
     * Indicates whether calibrator is currently running or no.
     *
     * @return true if calibrator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Gets Earth's magnetic model.
     *
     * @return Earth's magnetic model or null if not provided.
     */
    public WorldMagneticModel getMagneticModel() {
        return mMagneticModel;
    }

    /**
     * Sets Earth's magnetic model.
     *
     * @param magneticModel Earth's magnetic model to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setMagneticModel(final WorldMagneticModel magneticModel)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMagneticModel = magneticModel;
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
    public void setProgressDelta(final float progressDelta) throws LockedException {
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
    public void setConfidence(final double confidence) throws LockedException {
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
    public void setMaxIterations(final int maxIterations) throws LockedException {
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
    public void setResultRefined(final boolean refineResult) throws LockedException {
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
    public void setCovarianceKept(final boolean keepCovariance) throws LockedException {
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
     * Gets estimated magnetometer soft-iron matrix containing scale factors
     * and cross coupling errors.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unitless.
     *
     * @return estimated magnetometer soft-iron scale factors and cross coupling errors,
     * or null if not available.
     */
    public Matrix getEstimatedMm() {
        return mEstimatedMm;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    public Double getEstimatedSx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    public Double getEstimatedSy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    public Double getEstimatedSz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    public Double getEstimatedMxy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    public Double getEstimatedMxz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    public Double getEstimatedMyx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    public Double getEstimatedMyz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    public Double getEstimatedMzx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    public Double getEstimatedMzy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 1) : null;
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
     * Gets estimated covariance matrix for estimated calibration parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy.
     *
     * @return estimated covariance matrix for estimated position.
     */
    public Matrix getEstimatedCovariance() {
        return mEstimatedCovariance;
    }

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS_COMMON_Z_AXIS}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return mPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS_COMMON_Z_AXIS}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is less than {@link #MINIMUM_MEASUREMENTS_COMMON_Z_AXIS}.
     */
    public void setPreliminarySubsetSize(final int preliminarySubsetSize)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < MINIMUM_MEASUREMENTS_COMMON_Z_AXIS) {
            throw new IllegalArgumentException();
        }

        mPreliminarySubsetSize = preliminarySubsetSize;
    }

    /**
     * Estimates magnetometer calibration parameters containing soft-iron
     * scale factors and cross-coupling errors.
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
     * Creates a robust magnetometer calibrator.
     *
     * @param method robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator();
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator();
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator();
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator();
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator();
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        measurements);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        measurements);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        measurements);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        measurements);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        commonAxisUsed);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final WorldMagneticModel magneticModel,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        magneticModel);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        magneticModel);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        magneticModel);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        magneticModel);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        magneticModel);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param hardIron known hard-iron.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param hardIron known hard-iron.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final Matrix hardIron, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param hardIron  known hard-iron.
     * @param initialMm initial soft-iron matrix containing scale factors
     *                  and cross coupling errors.
     * @param method    robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron, initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron, initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron, initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron, initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron, initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        measurements);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        measurements);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        measurements);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, measurements);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, commonAxisUsed);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final WorldMagneticModel magneticModel,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        magneticModel);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        magneticModel);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        magneticModel);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, magneticModel);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, magneticModel);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron, initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron, initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        hardIron, initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, hardIron, initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, hardIron, initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        initialMm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        initialMm, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, initialMm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, initialMm, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements,
                        hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements,
                        hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm,
                        listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm,
                        listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, hardIron, initialMm,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        initialMm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, hardIron,
                        initialMm, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, initialMm);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, initialMm);
        }
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores,
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case LMedS:
                return new LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case MSAC:
                return new MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        position, measurements, commonAxisUsed, hardIron,
                        initialMm, listener);
            case PROSAC:
                return new PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, initialMm, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
                        qualityScores, position, measurements, commonAxisUsed,
                        hardIron, initialMm, listener);
        }
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final WorldMagneticModel magneticModel) {
        return create(magneticModel, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param hardIron known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final double[] hardIron) {
        return create(hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param hardIron known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final Matrix hardIron) {
        return create(hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param hardIron  known hard-iron.
     * @param initialMm initial soft-iron matrix containing scale factors
     *                  and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final Matrix hardIron, final Matrix initialMm) {
        return create(hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position) {
        return create(position, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(position, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        return create(position, measurements, commonAxisUsed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron) {
        return create(position, measurements, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, hardIron, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron) {
        return create(position, measurements, commonAxisUsed, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed,
                hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron) {
        return create(position, measurements, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, hardIron, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron) {
        return create(position, measurements, commonAxisUsed,
                hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed,
                hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm) {
        return create(position, measurements, hardIron, initialMm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, hardIron, initialMm,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm) {
        return create(position, measurements, commonAxisUsed,
                hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed,
                hardIron, initialMm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position) {
        return create(position, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(position, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        return create(position, measurements, commonAxisUsed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron) {
        return create(position, measurements, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, hardIron,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron) {
        return create(position, measurements, commonAxisUsed,
                hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed,
                hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron) {
        return create(position, measurements, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, hardIron, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron) {
        return create(position, measurements, commonAxisUsed,
                hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed,
                hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm) {
        return create(position, measurements, hardIron, initialMm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, hardIron,
                initialMm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm) {
        return create(position, measurements, commonAxisUsed,
                hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed,
                hardIron, initialMm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Initializes calibrator and estimates magnetic flux density norm
     * at provided position and timestamp.
     *
     * @throws IOException if world magnetic model cannot be loaded.
     */
    protected void initialize() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator;
        if (mMagneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(mMagneticModel);
        } else {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }

        final NEDPosition position = getNedPosition();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, mYear);
        mMagneticDensityNorm = earthB.getNorm();
    }

    /**
     * Computes error of a preliminary result respect a given measurement.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final StandardDeviationBodyMagneticFluxDensity measurement,
            final Matrix preliminaryResult) {

        try {
            // The magnetometer model is:
            // mBmeas = bm + (I + Mm) * mBtrue

            // mBmeas - bm = (I + Mm) * mBtrue

            // mBtrue = (I + Mm)^-1 * (mBmeas - ba)

            // We know that ||mBtrue||should be equal to the magnitude of the
            // Earth magnetic field at provided location

            if (mIdentity == null) {
                mIdentity = Matrix.identity(
                        BodyMagneticFluxDensity.COMPONENTS,
                        BodyMagneticFluxDensity.COMPONENTS);
            }

            if (mTmp1 == null) {
                mTmp1 = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                        BodyMagneticFluxDensity.COMPONENTS);
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

            mIdentity.add(preliminaryResult, mTmp1);

            Utils.inverse(mTmp1, mTmp2);

            final BodyMagneticFluxDensity measuredMagneticFluxDensity =
                    measurement.getMagneticFluxDensity();
            final double bMeasX = measuredMagneticFluxDensity.getBx();
            final double bMeasY = measuredMagneticFluxDensity.getBy();
            final double bMeasZ = measuredMagneticFluxDensity.getBz();

            mTmp3.setElementAtIndex(0, bMeasX - mHardIronX);
            mTmp3.setElementAtIndex(1, bMeasY - mHardIronY);
            mTmp3.setElementAtIndex(2, bMeasZ - mHardIronZ);

            mTmp2.multiply(mTmp3, mTmp4);

            final double norm = Utils.normF(mTmp4);
            final double diff = mMagneticDensityNorm - norm;

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
            final int[] samplesIndices, final List<Matrix> solutions) {

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                new ArrayList<>();

        for (final int samplesIndex : samplesIndices) {
            measurements.add(mMeasurements.get(samplesIndex));
        }

        try {
            final Matrix result = getInitialMm();

            mInnerCalibrator.setHardIron(mHardIronX, mHardIronY, mHardIronZ);
            mInnerCalibrator.setInitialMm(result);
            mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
            mInnerCalibrator.setPosition(mPosition);
            mInnerCalibrator.setYear(mYear);
            mInnerCalibrator.setMeasurements(measurements);
            mInnerCalibrator.calibrate();

            result.copyFrom(mInnerCalibrator.getEstimatedMm());

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
    protected void attemptRefine(final Matrix preliminaryResult) {
        if (mRefineResult && mInliersData != null) {
            final BitSet inliers = mInliersData.getInliers();
            final int nSamples = mMeasurements.size();

            final List<StandardDeviationBodyMagneticFluxDensity> inlierMeasurements =
                    new ArrayList<>();
            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierMeasurements.add(mMeasurements.get(i));
                }
            }

            try {
                mInnerCalibrator.setHardIron(mHardIronX, mHardIronY, mHardIronZ);
                mInnerCalibrator.setInitialMm(preliminaryResult);
                mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mInnerCalibrator.setPosition(mPosition);
                mInnerCalibrator.setYear(mYear);
                mInnerCalibrator.setMeasurements(inlierMeasurements);
                mInnerCalibrator.calibrate();

                mEstimatedMm = mInnerCalibrator.getEstimatedMm();

                if (mKeepCovariance) {
                    mEstimatedCovariance = mInnerCalibrator.getEstimatedCovariance();
                } else {
                    mEstimatedCovariance = null;
                }

                mEstimatedMse = mInnerCalibrator.getEstimatedMse();

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                mEstimatedCovariance = null;
                mEstimatedMm = preliminaryResult;
                mEstimatedMse = 0.0;
            }
        } else {
            mEstimatedCovariance = null;
            mEstimatedMm = preliminaryResult;
            mEstimatedMse = 0.0;
        }
    }

    /**
     * Converts a time instance expressed in milliseconds since epoch time
     * (January 1st, 1970 at midnight) to a decimal year.
     *
     * @param timestampMillis milliseconds value to be converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final Long timestampMillis) {
        if (timestampMillis == null) {
            return null;
        }

        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestampMillis);
        return convertTime(calendar);
    }

    /**
     * Converts a time instant contained ina date object to a
     * decimal year.
     *
     * @param date a time instance to be converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final Date date) {
        if (date == null) {
            return null;
        }

        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(date);
        return convertTime(calendar);
    }

    /**
     * Converts a time instant contained in a gregorian calendar to a
     * decimal year.
     *
     * @param calendar calendar containing a specific instant to be
     *                 converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final GregorianCalendar calendar) {
        if (calendar == null) {
            return null;
        }

        return WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
    }

    /**
     * Converts provided ECEF position to position expressed in NED
     * coordinates.
     *
     * @param position ECEF position to be converted.
     * @return converted position expressed in NED coordinates.
     */
    private static NEDPosition convertPosition(final ECEFPosition position) {
        final NEDVelocity velocity = new NEDVelocity();
        final NEDPosition result = new NEDPosition();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                position.getX(), position.getY(), position.getZ(),
                0.0, 0.0, 0.0, result, velocity);
        return result;
    }

    /**
     * Converts magnetic flux density value and unit to Teslas.
     *
     * @param value magnetic flux density value.
     * @param unit unit of magnetic flux density value.
     * @return converted value.
     */
    private static double convertMagneticFluxDensity(final double value, final MagneticFluxDensityUnit unit) {
        return MagneticFluxDensityConverter.convert(value, unit,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Converts magnetic flux density instance to Teslas.
     *
     * @param magneticFluxDensity magnetic flux density instance to be converted.
     * @return converted value.
     */
    private static double convertMagneticFluxDensity(final MagneticFluxDensity magneticFluxDensity) {
        return convertMagneticFluxDensity(magneticFluxDensity.getValue().doubleValue(),
                magneticFluxDensity.getUnit());
    }
}
