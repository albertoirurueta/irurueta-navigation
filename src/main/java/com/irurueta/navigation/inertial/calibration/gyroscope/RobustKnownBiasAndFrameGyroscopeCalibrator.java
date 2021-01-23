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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate gyroscope
 * cross couplings and scaling factors along with G-dependent
 * cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * This estimator assumes that biases are known.
 * <p>
 * To use this calibrator at least 6 measurements at different known frames must
 * be provided. In other words, accelerometer and gyroscope (i.e. body kinematics)
 * samples must be obtained at 6 different positions, orientations and velocities
 * (although typically velocities are always zero).
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
public abstract class RobustKnownBiasAndFrameGyroscopeCalibrator implements
        GyroscopeNonLinearCalibrator, KnownBiasGyroscopeCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 6;

    /**
     * Indicates that by default a linear calibrator is used for preliminary solution estimation.
     * The result obtained on each preliminary solution might be later refined.
     */
    public static final boolean DEFAULT_USE_LINEAR_CALIBRATOR = true;

    /**
     * Indicates that by default preliminary solutions are refined.
     */
    public static final boolean DEFAULT_REFINE_PRELIMINARY_SOLUTIONS = false;

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
     * Contains a list of body kinematics measurements taken at different
     * frames (positions, orientations and velocities) and containing the standard
     * deviations of accelerometer and gyroscope measurements.
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     */
    protected List<StandardDeviationFrameBodyKinematics> mMeasurements;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownBiasAndFrameGyroscopeCalibratorListener mListener;

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
    protected int mPreliminarySubsetSize = MINIMUM_MEASUREMENTS;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Known x coordinate of gyroscope bias expressed in radians per second (rad/s).
     */
    private double mBiasX;

    /**
     * Known y coordinate of gyroscope bias expressed in radians per second (rad/s).
     */
    private double mBiasY;

    /**
     * Known z coordinate of gyroscope bias expressed in radians per second (rad/s).
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
     * Initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     */
    private Matrix mInitialGg;

    /**
     * Indicates whether a linear calibrator is used or not for preliminary
     * solutions.
     */
    private boolean mUseLinearCalibrator = DEFAULT_USE_LINEAR_CALIBRATOR;

    /**
     * Indicates whether preliminary solutions must be refined after an initial linear solution
     * is found.
     */
    private boolean mRefinePreliminarySolutions = DEFAULT_REFINE_PRELIMINARY_SOLUTIONS;

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
     * Estimated chi square value.
     */
    private double mEstimatedChiSq;

    /**
     * Estimated mean square error respect to provided measurements.
     */
    private double mEstimatedMse;

    /**
     * A linear least squares calibrator.
     */
    private final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator mLinearCalibrator =
            new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

    /**
     * A non-linear least squares calibrator.
     */
    private final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator mNonLinearCalibrator =
            new KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator();

    /**
     * Constructor.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator() {
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
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this();
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        this();
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final boolean commonAxisUsed) {
        this();
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @param biasY known y coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @param biasZ known z coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        this();
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param biasY    known y coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param biasZ    known z coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(measurements);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(commonAxisUsed, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias.
     * @param biasY known y coordinate of gyroscope bias.
     * @param biasZ known z coordinate of gyroscope bias.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        this();
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias.
     * @param biasY    known y coordinate of gyroscope bias.
     * @param biasZ    known z coordinate of gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        this(measurements);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(commonAxisUsed, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param bias known gyroscope bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] bias) {
        this();
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias) {
        this(measurements);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] bias, final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(commonAxisUsed, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias known gyroscope bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final Matrix bias) {
        this();
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias) {
        this(measurements);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final Matrix bias, final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(commonAxisUsed, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public RobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetBias(bias);
    }

    /**
     * Gets known x coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return x coordinate of gyroscope bias.
     */
    @Override
    public double getBiasX() {
        return mBiasX;
    }

    /**
     * Sets known x coordinate of gyroscope bias expressed in radians persecond
     * (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasX(final double biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
    }

    /**
     * Gets known y coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return y coordinate of gyroscope bias.
     */
    @Override
    public double getBiasY() {
        return mBiasY;
    }

    /**
     * Sets known y coordinate of gyroscope bias expressed in radians per second
     * (rad/s).)
     *
     * @param biasY y coordinate of gyroscope bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasY(final double biasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasY = biasY;
    }

    /**
     * Gets known z coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return z coordinate of gyroscope bias.
     */
    @Override
    public double getBiasZ() {
        return mBiasZ;
    }

    /**
     * Sets known z coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasZ(final double biasZ) throws LockedException {
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
    @Override
    public AngularSpeed getBiasAngularSpeedX() {
        return new AngularSpeed(mBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known x-coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(mBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known x-coordinate of gyroscope bias.
     *
     * @param biasX x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final AngularSpeed biasX) throws LockedException {
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
    @Override
    public AngularSpeed getBiasAngularSpeedY() {
        return new AngularSpeed(mBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known y-coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(mBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known y-coordinate of gyroscope bias.
     *
     * @param biasY y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final AngularSpeed biasY) throws LockedException {
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
    @Override
    public AngularSpeed getBiasAngularSpeedZ() {
        return new AngularSpeed(mBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known z-coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known z-coordinate of gyroscope bias.
     *
     * @param biasZ z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final AngularSpeed biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Sets known gyroscope bias coordinates expressed in radians per second (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
     * @param biasY y coordinate of gyroscope bias.
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(
            final double biasX, final double biasY, final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Sets known gyroscope bias coordinates.
     *
     * @param biasX x coordinate of gyroscope bias.
     * @param biasY y coordinate of gyroscope bias.
     * @param biasZ z coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(final AngularSpeed biasX, final AngularSpeed biasY,
                                   final AngularSpeed biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        internalSetBiasCoordinates(biasX, biasY, biasZ);
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
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of known bias.
     */
    @Override
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
     * Sets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known gyroscope bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void setBias(final double[] bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        internalSetBias(bias);
    }

    /**
     * Gets known gyroscope bias as a column matrix.
     *
     * @return known gyroscope bias as a column matrix.
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
     * Gets known gyroscope bias as a column matrix.
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
     * Sets known gyroscope bias as a column matrix.
     *
     * @param bias gyroscope bias to be set.
     * @throws LockedException          if calibrator is currently running
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setBias(final Matrix bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        internalSetBias(bias);
    }

    /**
     * Gets initial x scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x scaling factor.
     */
    @Override
    public double getInitialSx() {
        return mInitialSx;
    }

    /**
     * Sets initial x scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSx initial x scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
    @Override
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
     * Gets initial scale factors and cross coupling errors matrix.
     *
     * @return initial scale factors and cross coupling errors matrix.
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
     * Gets initial scale factors and cross coupling errors matrix.
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
     * Sets initial scale factors and cross coupling errors matrix.
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
     * Gets a list of body kinematics measurements taken at different
     * frames (positions, orientations and velocities) and containing the standard
     * deviations of accelerometer and gyroscope measurements.
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate the a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     *
     * @return a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     */
    public List<StandardDeviationFrameBodyKinematics> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets a list of body kinematics measurements taken at different
     * frames (positions, orientations and velocities) and containing the standard
     * deviations of accelerometer and gyroscope measurements.
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate the a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     *
     * @param measurements collection of body kinematics measurements taken at different
     *                     frames (positions, orientations and velocities).
     * @throws LockedException if calibrator is currently running.
     */
    public void setMeasurements(
            final List<StandardDeviationFrameBodyKinematics> measurements)
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
    public void setCommonAxisUsed(final boolean commonAxisUsed)
            throws LockedException {
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
    public RobustKnownBiasAndFrameGyroscopeCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mMeasurements != null && mMeasurements.size() >= MINIMUM_MEASUREMENTS;
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
     * Indicates whether a linear calibrator is used or not for preliminary
     * solutions.
     *
     * @return indicates whether a linear calibrator is used or not for
     * preliminary solutions.
     */
    public boolean isLinearCalibratorUsed() {
        return mUseLinearCalibrator;
    }

    /**
     * Specifies whether a linear calibrator is used or not for preliminary
     * solutions.
     *
     * @param linearCalibratorUsed indicates whether a linear calibrator is used
     *                             or not for preliminary solutions.
     * @throws LockedException if calibrator is currently running.
     */
    public void setLinearCalibratorUsed(final boolean linearCalibratorUsed)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mUseLinearCalibrator = linearCalibratorUsed;
    }

    /**
     * Indicates whether preliminary solutions must be refined after an initial linear solution is found.
     * If no initial solution is found using a linear solver, a non linear solver will be
     * used regardless of this value using an average solution as the initial value to be
     * refined.
     *
     * @return true if preliminary solutions must be refined after an initial linear solution, false
     * otherwise.
     */
    public boolean isPreliminarySolutionRefined() {
        return mRefinePreliminarySolutions;
    }

    /**
     * Specifies whether preliminary solutions must be refined after an initial linear solution is found.
     * If no initial solution is found using a linear solver, a non linear solver will be
     * used regardless of this value using an average solution as the initial value to be
     * refined.
     *
     * @param preliminarySolutionRefined true if preliminary solutions must be refined after an
     *                                   initial linear solution, false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPreliminarySolutionRefined(
            final boolean preliminarySolutionRefined)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mRefinePreliminarySolutions = preliminarySolutionRefined;
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
    public void setProgressDelta(
            final float progressDelta) throws LockedException {
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
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return mEstimatedMg != null ?
                mEstimatedMg.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
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
     * Gets estimated covariance matrix for estimated calibration solution.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32, gg13, gg23, gg33.
     * This is only available when result has been refined and covariance
     * is kept.
     *
     * @return estimated covariance matrix for estimated position.
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
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return mPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is less than {@link #MINIMUM_MEASUREMENTS}.
     */
    public void setPreliminarySubsetSize(
            final int preliminarySubsetSize) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < MINIMUM_MEASUREMENTS) {
            throw new IllegalArgumentException();
        }

        mPreliminarySubsetSize = preliminarySubsetSize;
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
    public abstract void calibrate() throws LockedException, NotReadyException,
            CalibrationException;

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
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator();
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator();
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator();
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator();
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator();
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(measurements);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(measurements);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(measurements);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(measurements);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param biasX  known x coordinate of gyroscope bias expressed in radians per
     *               second (rad/s).
     * @param biasY  known y coordinate of gyroscope bias expressed in radians per
     *               second (rad/s).
     * @param biasZ  known z coordinate of gyroscope bias expressed in radians per
     *               second (rad/s).
     * @param method robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param biasX    known x coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param biasY    known y coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param biasZ    known z coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
        }
    }

    /**
     * Creaates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param biasX  known x coordinate of gyroscope bias.
     * @param biasY  known y coordinate of gyroscope bias.
     * @param biasZ  known z coordinate of gyroscope bias.
     * @param method robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param biasX    known x coordinate of gyroscope bias.
     * @param biasY    known y coordinate of gyroscope bias.
     * @param biasZ    known z coordinate of gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        biasX, biasY, biasZ, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param bias   known gyroscope bias.
     * @param method robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param bias     known gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] bias, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param bias   known gyroscope bias.
     * @param method robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final Matrix bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param bias     known gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         a robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasY         known y coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasZ         known z coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final double biasX, final double biasY,
            final double biasZ, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasY         known y coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasZ         known z coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final double biasX, final double biasY,
            final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, listener);
        }
    }

    /**
     * Cretes a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasY         known y coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasZ         known z coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasY         known y coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasZ         known z coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final double biasX, final double biasY,
            final double biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ,
                        commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ,
                        commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ,
                        commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ,
                        commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of gyroscope bias.
     * @param biasY         known y coordinate of gyroscope bias.
     * @param biasZ         known z coordinate of gyroscope bias.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of gyroscope bias.
     * @param biasY         known y coordinate of gyroscope bias.
     * @param biasZ         known z coordinate of gyroscope bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of gyroscope bias.
     * @param biasY         known y coordinate of gyroscope bias.
     * @param biasZ         known z coordinate of gyroscope bias.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of gyroscope bias.
     * @param biasY         known y coordinate of gyroscope bias.
     * @param biasZ         known z coordinate of gyroscope bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(biasX,
                        biasY, biasZ, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, biasX, biasY, biasZ, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyrocope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ,
                        commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ,
                        commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ,
                        commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, biasX, biasY, biasZ,
                        commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known gyroscope bias.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final double[] bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known gyroscope bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known gyroscope bias.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known gyroscope bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(measurements,
                        bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(measurements,
                        bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(measurements,
                        bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final double[] bias,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final double[] bias,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(qualityScores,
                        measurements, bias, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(qualityScores,
                        measurements, bias, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known gyroscope bias.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final Matrix bias,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known gyroscope bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known gyroscope bias.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known gyroscope bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final Matrix bias,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        bias, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores, final Matrix bias,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(bias,
                        commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, bias, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, commonAxisUsed);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        measurements, bias, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
                        qualityScores, measurements, bias, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        return create(measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param biasX known x coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @param biasY known y coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @param biasZ known z coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double biasX, final double biasY, final double biasZ) {
        return create(biasX, biasY, biasZ, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param biasX    known x coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param biasY    known y coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param biasZ    known z coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(biasX, biasY, biasZ, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        return create(measurements, biasX, biasY, biasZ, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, biasX, biasY, biasZ, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        return create(biasX, biasY, biasZ, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(biasX, biasY, biasZ, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        return create(measurements, biasX, biasY, biasZ, commonAxisUsed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, biasX, biasY, biasZ, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param biasX known x coordinate of gyroscope bias.
     * @param biasY known y coordinate of gyroscope bias.
     * @param biasZ known z coordinate of gyroscope bias.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ) {
        return create(biasX, biasY, biasZ, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param biasX    known x coordinate of gyroscope bias.
     * @param biasY    known y coordinate of gyroscope bias.
     * @param biasZ    known z coordinate of gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(biasX, biasY, biasZ, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ) {
        return create(measurements, biasX, biasY, biasZ, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, biasX, biasY, biasZ, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed) {
        return create(biasX, biasY, biasZ, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(biasX, biasY, biasZ, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed) {
        return create(measurements, biasX, biasY, biasZ, commonAxisUsed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, biasX, biasY, biasZ, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param bias known gyroscope bias.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] bias) {
        return create(bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param bias     known gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias) {
        return create(measurements, bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] bias, final boolean commonAxisUsed) {
        return create(bias, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(bias, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed) {
        return create(measurements, bias, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, bias, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param bias known gyroscope bias.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final Matrix bias) {
        return create(bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param bias     known gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias) {
        return create(measurements, bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final Matrix bias, final boolean commonAxisUsed) {
        return create(bias, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(bias, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed) {
        return create(measurements, bias, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust estimator method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        return create(measurements, bias, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Computes error of a preliminary result respect a given measurement.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final StandardDeviationFrameBodyKinematics measurement,
            final PreliminaryResult preliminaryResult) {
        // We know that measured angular rate is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [myx   sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [mzx   mzy    sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        final BodyKinematics measuredKinematics = measurement.getKinematics();
        final ECEFFrame ecefFrame = measurement.getFrame();
        final ECEFFrame previousEcefFrame = measurement.getPreviousFrame();
        final double timeInterval = measurement.getTimeInterval();

        final BodyKinematics expectedKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, ecefFrame,
                        previousEcefFrame);

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

        try {
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

            final double diffX = angularRateMeasX2 - angularRateMeasX1;
            final double diffY = angularRateMeasY2 - angularRateMeasY1;
            final double diffZ = angularRateMeasZ2 - angularRateMeasZ1;

            return Math.sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

        } catch (final WrongSizeException e) {
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
            final int[] samplesIndices, final List<PreliminaryResult> solutions) {

        final List<StandardDeviationFrameBodyKinematics> measurements = new ArrayList<>();

        for (final int samplesIndex : samplesIndices) {
            measurements.add(mMeasurements.get(samplesIndex));
        }

        try {
            final PreliminaryResult result = new PreliminaryResult();
            result.mEstimatedMg = getInitialMg();
            result.mEstimatedGg = getInitialGg();

            if (mUseLinearCalibrator) {
                mLinearCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mLinearCalibrator.setMeasurements(measurements);
                mLinearCalibrator.setBiasCoordinates(mBiasX, mBiasY, mBiasZ);
                mLinearCalibrator.calibrate();

                result.mEstimatedMg = mLinearCalibrator.getEstimatedMg();
                result.mEstimatedGg = mLinearCalibrator.getEstimatedGg();
            }

            if (mRefinePreliminarySolutions) {
                mNonLinearCalibrator.setInitialMg(result.mEstimatedMg);
                mNonLinearCalibrator.setInitialGg(result.mEstimatedGg);
                mNonLinearCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mNonLinearCalibrator.setMeasurements(measurements);
                mNonLinearCalibrator.setBiasCoordinates(mBiasX, mBiasY, mBiasZ);
                mNonLinearCalibrator.calibrate();

                result.mEstimatedMg = mNonLinearCalibrator.getEstimatedMg();
                result.mEstimatedGg = mNonLinearCalibrator.getEstimatedGg();

                if (mKeepCovariance) {
                    result.mCovariance = mNonLinearCalibrator.getEstimatedCovariance();
                } else {
                    result.mCovariance = null;
                }

                result.mEstimatedMse = mNonLinearCalibrator.getEstimatedMse();
                result.mEstimatedChiSq = mNonLinearCalibrator.getEstimatedChiSq();

            } else {
                result.mCovariance = null;
                result.mEstimatedMse = 0.0;
                result.mEstimatedChiSq = 0.0;
            }

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

            final List<StandardDeviationFrameBodyKinematics> inlierMeasurements =
                    new ArrayList<>();
            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierMeasurements.add(mMeasurements.get(i));
                }
            }

            try {
                mNonLinearCalibrator.setInitialMg(preliminaryResult.mEstimatedMg);
                mNonLinearCalibrator.setInitialGg(preliminaryResult.mEstimatedGg);
                mNonLinearCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mNonLinearCalibrator.setMeasurements(inlierMeasurements);
                mNonLinearCalibrator.setBiasCoordinates(mBiasX, mBiasY, mBiasZ);
                mNonLinearCalibrator.calibrate();

                mEstimatedMg = mNonLinearCalibrator.getEstimatedMg();
                mEstimatedGg = mNonLinearCalibrator.getEstimatedGg();

                if (mKeepCovariance) {
                    mEstimatedCovariance = mNonLinearCalibrator.getEstimatedCovariance();
                } else {
                    mEstimatedCovariance = null;
                }
                mEstimatedMse = mNonLinearCalibrator.getEstimatedMse();
                mEstimatedChiSq = mNonLinearCalibrator.getEstimatedChiSq();

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                mEstimatedCovariance = preliminaryResult.mCovariance;
                mEstimatedMg = preliminaryResult.mEstimatedMg;
                mEstimatedGg = preliminaryResult.mEstimatedGg;
                mEstimatedMse = preliminaryResult.mEstimatedMse;
                mEstimatedChiSq = preliminaryResult.mEstimatedChiSq;
            }
        } else {
            mEstimatedCovariance = preliminaryResult.mCovariance;
            mEstimatedMg = preliminaryResult.mEstimatedMg;
            mEstimatedGg = preliminaryResult.mEstimatedGg;
            mEstimatedMse = preliminaryResult.mEstimatedMse;
            mEstimatedChiSq = preliminaryResult.mEstimatedChiSq;
        }
    }

    /**
     * Internally sets bias coordinates.
     *
     * @param biasX known x coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @param biasY known y coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @param biasZ known z coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     */
    private void internalSetBiasCoordinates(
            final double biasX, final double biasY, final double biasZ) {
        mBiasX = biasX;
        mBiasY = biasY;
        mBiasZ = biasZ;
    }

    /**
     * Internally sets bias coordinates.
     *
     * @param biasX known x coordinate of gyroscope bias.
     * @param biasY known y coordinate of gyroscope bias.
     * @param biasZ known z coordinate of gyroscope bias.
     */
    private void internalSetBiasCoordinates(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        mBiasX = convertAngularSpeed(biasX);
        mBiasY = convertAngularSpeed(biasY);
        mBiasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Internally sets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known gyroscope bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    private void internalSetBias(final double[] bias) {
        if (bias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        mBiasX = bias[0];
        mBiasY = bias[1];
        mBiasZ = bias[2];
    }

    /**
     * Internally sets known gyrocope bias as a column matrix.
     * Matrix values are expressed in radians per second (rad/s).
     *
     * @param bias gyroscope bias to be set.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    private void internalSetBias(final Matrix bias) {
        if (bias.getRows() != BodyKinematics.COMPONENTS
                || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mBiasX = bias.getElementAtIndex(0);
        mBiasY = bias.getElementAtIndex(1);
        mBiasZ = bias.getElementAtIndex(2);
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

        /**
         * Estimated chi square value.
         */
        private double mEstimatedChiSq;
    }
}
