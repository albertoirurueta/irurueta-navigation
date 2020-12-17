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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSTightlyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate accelerometer
 * biases, cross couplings and scaling factors.
 * <p>
 * To use this calibrator at least 4 measurements at different known frames must
 * be provided. In other words, accelerometer samples must be obtained at 4
 * different positions, orientations and velocities (although typically velocities are
 * always zero).
 * <p>
 * Measured specific force is assumed to follow the model shown below:
 * <pre>
 *     fmeas = ba + (I + Ma) * ftrue + w
 * </pre>
 * Where:
 * - fmeas is the measured specific force. This is a 3x1 vector.
 * - ba is accelerometer bias. Ideally, on a perfect accelerometer, this should be a
 * 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Ma is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect accelerometer, this should be a 3x3 zero matrix.
 * - ftrue is ground-trush specific force.
 * - w is measurement noise.
 */
public abstract class RobustKnownFrameAccelerometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 4;

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
    protected RobustKnownFrameAccelerometerCalibratorListener mListener;

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
    protected int mPreliminarySubsetSize = MINIMUM_MEASUREMENTS;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mInitialBiasX;

    /**
     * Initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mInitialBiasY;

    /**
     * Initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mInitialBiasZ;

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
     * Estimated accelerometer biases for each IMU axis expressed in meter per squared
     * second (m/s^2).
     */
    private double[] mEstimatedBiases;

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
     * A linear least squares calibrator.
     */
    private final KnownFrameAccelerometerLinearLeastSquaresCalibrator mLinearCalibrator =
            new KnownFrameAccelerometerLinearLeastSquaresCalibrator();

    /**
     * A non-linear least squares calibrator.
     */
    private final KnownFrameAccelerometerNonLinearLeastSquaresCalibrator mNonLinearCalibrator =
            new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator();

    /**
     * Constructor.
     */
    public RobustKnownFrameAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustKnownFrameAccelerometerCalibrator(
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    public RobustKnownFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
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
    public RobustKnownFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public RobustKnownFrameAccelerometerCalibrator(final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public RobustKnownFrameAccelerometerCalibrator(
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
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
    public RobustKnownFrameAccelerometerCalibrator(
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
    public RobustKnownFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solutions.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial x-coordinate of accelerometer bias.
     */
    public double getInitialBiasX() {
        return mInitialBiasX;
    }

    /**
     * Sets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasX(final double initialBiasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = initialBiasX;
    }

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial y-coordinate of accelerometer bias.
     */
    public double getInitialBiasY() {
        return mInitialBiasY;
    }

    /**
     * Sets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasY(final double initialBiasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasY = initialBiasY;
    }

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial z-coordinate of accelerometer bias.
     */
    public double getInitialBiasZ() {
        return mInitialBiasZ;
    }

    /**
     * Sets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasZ(final double initialBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasZ = initialBiasZ;
    }

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x-coordinate of accelerometer bias.
     */
    public Acceleration getInitialBiasXAsAcceleration() {
        return new Acceleration(mInitialBiasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasXAsAcceleration(final Acceleration result) {
        result.setValue(mInitialBiasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasX(final Acceleration initialBiasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = convertAcceleration(initialBiasX);
    }

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial y-coordinate of accelerometer bias.
     */
    public Acceleration getInitialBiasYAsAcceleration() {
        return new Acceleration(mInitialBiasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasYAsAcceleration(final Acceleration result) {
        result.setValue(mInitialBiasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasY(final Acceleration initialBiasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasY = convertAcceleration(initialBiasY);
    }

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial z-coordinate of accelerometer bias.
     */
    public Acceleration getInitialBiasZAsAcceleration() {
        return new Acceleration(mInitialBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasZAsAcceleration(final Acceleration result) {
        result.setValue(mInitialBiasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasZ(final Acceleration initialBiasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasZ = convertAcceleration(initialBiasZ);
    }

    /**
     * Sets initial bias coordinates of accelerometer used to find a solution
     * expressed in meters per squared second (m/s^2).
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBias(final double initialBiasX, final double initialBiasY,
                               final double initialBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = initialBiasX;
        mInitialBiasY = initialBiasY;
        mInitialBiasZ = initialBiasZ;
    }

    /**
     * Sets initial bias coordinates of accelerometer used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBias(final Acceleration initialBiasX,
                               final Acceleration initialBiasY,
                               final Acceleration initialBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = convertAcceleration(initialBiasX);
        mInitialBiasY = convertAcceleration(initialBiasY);
        mInitialBiasZ = convertAcceleration(initialBiasZ);
    }

    /**
     * Gets initial bias coordinates of accelerometer used to find a solution.
     *
     * @return initial bias coordinates.
     */
    public AccelerationTriad getInitialBiasAsTriad() {
        return new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND,
                mInitialBiasX, mInitialBiasY, mInitialBiasZ);
    }

    /**
     * Gets initial bias coordinates of accelerometer used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialBiasAsTriad(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(
                mInitialBiasX, mInitialBiasY, mInitialBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial bias coordinates of accelerometer used to find a solution.
     *
     * @param initialBias initial bias coordinates to be set.
     */
    public void setInitialBias(final AccelerationTriad initialBias) {
        mInitialBiasX = convertAcceleration(
                initialBias.getValueX(), initialBias.getUnit());
        mInitialBiasY = convertAcceleration(
                initialBias.getValueY(), initialBias.getUnit());
        mInitialBiasZ = convertAcceleration(
                initialBias.getValueZ(), initialBias.getUnit());
    }

    /**
     * Gets initial x scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x scaling factor.
     */
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
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return array containing coordinates of initial bias.
     */
    public double[] getInitialBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getInitialBias(result);
        return result;
    }

    /**
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     * This is only taken into account if non-linear preliminary solutions are used.
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
     * Sets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setInitialBias(final double[] initialBias) throws LockedException {
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
     * Gets initial bias to be used to find a solution as a column matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     * Values are expressed in meters per squared second (m/s^2).
     *
     * @return initial bias to be used to find a solution as a column matrix.
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
     * Gets initial bias to be used to find a solution as a column matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     * Values are expressed in meters per squared second (m/s^2).
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
     * Sets initial bias to be used to find a solution as an array.
     * This is only taken into account if non-linear preliminary solutions are used.
     * Values are expressed in meters per squared second (m/s^2).
     *
     * @param initialBias initial bias to find a solution.
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
    public RobustKnownFrameAccelerometerCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final RobustKnownFrameAccelerometerCalibratorListener listener)
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
    public boolean isReady() {
        return mMeasurements != null && mMeasurements.size() >= MINIMUM_MEASUREMENTS;
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
            final boolean preliminarySolutionRefined) throws LockedException {
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
    public void setConfidence(
            final double confidence) throws LockedException {
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
    public void setMaxIterations(
            final int maxIterations) throws LockedException {
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
    public void setResultRefined(
            final boolean refineResult) throws LockedException {
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
    public void setCovarianceKept(
            final boolean keepCovariance) throws LockedException {
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
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return array containing x,y,z components of estimated accelerometer biases.
     */
    public double[] getEstimatedBiases() {
        return mEstimatedBiases;
    }

    /**
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @param result instance where estimated accelerometer biases will be stored.
     * @return true if result instance was updated, false otherwise (when estimation
     * is not yet available).
     */
    public boolean getEstimatedBiases(final double[] result) {
        if (mEstimatedBiases != null) {
            System.arraycopy(mEstimatedBiases, 0, result,
                    0, mEstimatedBiases.length);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets column matrix containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return column matrix containing x,y,z components of estimated accelerometer
     * biases
     */
    public Matrix getEstimatedBiasesAsMatrix() {
        return mEstimatedBiases != null ? Matrix.newFromArray(mEstimatedBiases) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    public boolean getEstimatedBiasesAsMatrix(final Matrix result)
            throws WrongSizeException {
        if (mEstimatedBiases != null) {
            result.fromArray(mEstimatedBiases);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets x coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return x coordinate of estimated accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFx() {
        return mEstimatedBiases != null ? mEstimatedBiases[0] : null;
    }

    /**
     * Gets y coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return y coordinate of estimated accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFy() {
        return mEstimatedBiases != null ? mEstimatedBiases[1] : null;
    }

    /**
     * Gets z coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return z coordinate of estimated accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFz() {
        return mEstimatedBiases != null ? mEstimatedBiases[2] : null;
    }

    /**
     * Gets x coordinate of estimated accelerometer bias.
     *
     * @return x coordinate of estimated accelerometer bias or null if not available.
     */
    public Acceleration getEstimatedBiasFxAsAcceleration() {
        return mEstimatedBiases != null ?
                new Acceleration(mEstimatedBiases[0],
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets x coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    public boolean getEstimatedBiasFxAsAcceleration(final Acceleration result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[0]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     *
     * @return y coordinate of estimated accelerometer bias or null if not available.
     */
    public Acceleration getEstimatedBiasFyAsAcceleration() {
        return mEstimatedBiases != null ?
                new Acceleration(mEstimatedBiases[1],
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    public boolean getEstimatedBiasFyAsAcceleration(final Acceleration result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[1]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     *
     * @return z coordinate of estimated accelerometer bias or null if not available.
     */
    public Acceleration getEstimatedBiasFzAsAcceleration() {
        return mEstimatedBiases != null ?
                new Acceleration(mEstimatedBiases[2],
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    public boolean getEstimatedBiasFzAsAcceleration(final Acceleration result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[2]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated accelerometer bias.
     *
     * @return estimated accelerometer bias or null if not available.
     */
    public AccelerationTriad getEstimatedBiasAsTriad() {
        return mEstimatedBiases != null ?
                new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                        mEstimatedBiases[0], mEstimatedBiases[1], mEstimatedBiases[2]) : null;
    }

    /**
     * Gets estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated accelerometer bias is available and result was
     * modified, false otherwise.
     */
    public boolean getEstimatedBiasAsTriad(final AccelerationTriad result) {
        if (mEstimatedBiases != null) {
            result.setValueCoordinatesAndUnit(
                    mEstimatedBiases[0], mEstimatedBiases[1], mEstimatedBiases[2],
                    AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
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
     * Gets estimated covariance matrix for estimated calibration solution.
     * This is only available when result has been refined and covariance
     * is kept.
     *
     * @return estimated covariance matrix for estimated position.
     */
    public Matrix getEstimatedCovariance() {
        return mEstimatedCovariance;
    }

    /**
     * Gets variance of estimated x coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated x coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFxVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     *
     * @return standard deviation of estimated x coordinate of accelerometer bias or null if not
     * available.
     */
    public Double getEstimatedBiasFxStandardDeviation() {
        final Double variance = getEstimatedBiasFxVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of accelerometer bias.
     *
     * @return standard deviation of estimated x coordinate of accelerometer bias or null if not
     * available.
     */
    public Acceleration getEstimatedBiasFxStandardDeviationAsAcceleration() {
        return mEstimatedCovariance != null ?
                new Acceleration(getEstimatedBiasFxStandardDeviation(), AccelerationUnit.METERS_PER_SQUARED_SECOND) :
                null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated x coordinate of accelerometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasFxStandardDeviationAsAcceleration(final Acceleration result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasFxStandardDeviation());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated y coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated y coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFyVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     *
     * @return standard deviation of estimated y coordinate of accelerometer bias or null if not
     * available.
     */
    public Double getEstimatedBiasFyStandardDeviation() {
        final Double variance = getEstimatedBiasFyVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of accelerometer bias.
     *
     * @return standard deviation of estimated y coordinate of accelerometer bias or null if not
     * available.
     */
    public Acceleration getEstimatedBiasFyStandardDeviationAsAcceleration() {
        return mEstimatedCovariance != null ?
                new Acceleration(getEstimatedBiasFyStandardDeviation(), AccelerationUnit.METERS_PER_SQUARED_SECOND) :
                null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated y coordinate of accelerometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasFyStandardDeviationAsAcceleration(final Acceleration result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasFyStandardDeviation());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated z coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated z coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFzVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     *
     * @return standard deviation of estimated z coordinate of accelerometer bias or null if not
     * available.
     */
    public Double getEstimatedBiasFzStandardDeviation() {
        final Double variance = getEstimatedBiasFzVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of accelerometer bias.
     *
     * @return standard deviation of estimated z coordinate of accelerometer bias or null if not
     * available.
     */
    public Acceleration getEstimatedBiasFzStandardDeviationAsAcceleration() {
        return mEstimatedCovariance != null ?
                new Acceleration(getEstimatedBiasFzStandardDeviation(), AccelerationUnit.METERS_PER_SQUARED_SECOND) :
                null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated z coordinate of accelerometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasFzStandardDeviationAsAcceleration(final Acceleration result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasFzStandardDeviation());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets standard deviation of estimated accelerometer bias coordinates.
     *
     * @return standard deviation of estimated accelerometer bias coordinates.
     */
    public AccelerationTriad getEstimatedBiasStandardDeviation() {
        return mEstimatedCovariance != null ?
                new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                        getEstimatedBiasFxStandardDeviation(),
                        getEstimatedBiasFyStandardDeviation(),
                        getEstimatedBiasFzStandardDeviation()) : null;
    }

    /**
     * Gets standard deviation of estimated accelerometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of accelerometer bias was available, false
     * otherwise.
     */
    public boolean getEstimatedBiasStandardDeviation(final AccelerationTriad result) {
        if (mEstimatedCovariance != null) {
            result.setValueCoordinatesAndUnit(
                    getEstimatedBiasFxStandardDeviation(),
                    getEstimatedBiasFyStandardDeviation(),
                    getEstimatedBiasFzStandardDeviation(),
                    AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets average of estimated standard deviation of accelerometer bias coordinates expressed
     * in meters per squared second (m/s^2).
     *
     * @return average of estimated standard deviation of accelerometer bias coordinates or null
     * if not available.
     */
    public Double getEstimatedBiasStandardDeviationAverage() {
        return mEstimatedCovariance != null ?
                (getEstimatedBiasFxStandardDeviation() +
                        getEstimatedBiasFyStandardDeviation() +
                        getEstimatedBiasFzStandardDeviation()) / 3.0 : null;
    }

    /**
     * Gets average of estimated standard deviation of accelerometer bias coordinates.
     *
     * @return average of estimated standard deviation of accelerometer bias coordinates or null.
     */
    public Acceleration getEstimatedBiasStandardDeviationAverageAsAcceleration() {
        return mEstimatedCovariance != null ?
                new Acceleration(getEstimatedBiasStandardDeviationAverage(),
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets average of estimated standard deviation of accelerometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if average of estimated standard deviation of accelerometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasStandardDeviationAverageAsAcceleration(final Acceleration result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasStandardDeviationAverage());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     * This can be used as the initial accelerometer bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of accelerometer bias or null
     * if not available.
     */
    public Double getEstimatedBiasStandardDeviationNorm() {
        return mEstimatedCovariance != null ?
                Math.sqrt(getEstimatedBiasFxVariance() + getEstimatedBiasFyVariance() + getEstimatedBiasFzVariance()) :
                null;
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias.
     * This can be used as the initial accelerometer bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of accelerometer bias or null
     * if not available.
     */
    public Acceleration getEstimatedBiasStandardDeviationNormAsAcceleration() {
        return mEstimatedCovariance != null ?
                new Acceleration(getEstimatedBiasStandardDeviationNorm(),
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias coordinates.
     * This can be used as the initial accelerometer bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @param result instance where result will be stored.
     * @return true if norm of estimated standard deviation of accelerometer bias is
     * available, false otherwise.
     */
    public boolean getEstimatedBiasStandardDeviationNormAsAcceleration(final Acceleration result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasStandardDeviationNorm());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
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
     * Estimates accelerometer calibration parameters containing bias, scale factors
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
    public static RobustKnownFrameAccelerometerCalibrator create(
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator();
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator();
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator();
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator();
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator();
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
    public static RobustKnownFrameAccelerometerCalibrator create(
            final RobustKnownFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(listener);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(listener);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(listener);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(measurements);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(measurements);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(measurements);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        measurements);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(measurements,
                        listener);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(measurements,
                        listener);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(measurements,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(measurements,
                        listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        measurements, listener);
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
    public static RobustKnownFrameAccelerometerCalibrator create(
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(
                        commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(commonAxisUsed,
                        listener);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(commonAxisUsed,
                        listener);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(commonAxisUsed,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(
                        commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(measurements,
                        commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(measurements,
                        commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator();
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator();
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator();
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(
                        qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        qualityScores);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final RobustKnownFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(listener);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(listener);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(listener);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(
                        qualityScores, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        qualityScores, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(measurements);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(measurements);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(measurements);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(qualityScores,
                        measurements);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        qualityScores, measurements);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(measurements,
                        listener);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(measurements,
                        listener);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(measurements,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(qualityScores,
                        measurements, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        qualityScores, measurements, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(qualityScores,
                        commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                        commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(commonAxisUsed,
                        listener);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(commonAxisUsed,
                        listener);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(commonAxisUsed,
                        listener);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(qualityScores,
                        commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                        commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(
                        qualityScores, measurements, commonAxisUsed);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        qualityScores, measurements, commonAxisUsed);
        }
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed, listener);
            case LMedS:
                return new LMedSRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed, listener);
            case MSAC:
                return new MSACRobustKnownFrameAccelerometerCalibrator(
                        measurements, commonAxisUsed, listener);
            case PROSAC:
                return new PROSACRobustKnownFrameAccelerometerCalibrator(
                        qualityScores, measurements, commonAxisUsed, listener);
            case PROMedS:
            default:
                return new PROMedSRobustKnownFrameAccelerometerCalibrator(
                        qualityScores, measurements, commonAxisUsed, listener);
        }
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        return create(measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        return create(commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        return create(measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        return create(measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        return create(qualityScores, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param listener      listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        return create(qualityScores, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final boolean commonAxisUsed) {
        return create(qualityScores, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        return create(qualityScores, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        return create(qualityScores, measurements, commonAxisUsed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameAccelerometerCalibrator create(
            final double[] qualityScores,
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final RobustKnownFrameAccelerometerCalibratorListener listener) {
        return create(qualityScores, measurements, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Computes error of a preliminary result respect a given measurement.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(final StandardDeviationFrameBodyKinematics measurement,
                                  final PreliminaryResult preliminaryResult) {
        // We know that measured specific force is:
        // fmeas = ba + (I + Ma) * ftrue

        // Hence:
        //  [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        //  [fmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        //  [fmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

        final BodyKinematics measuredKinematics = measurement.getKinematics();
        final ECEFFrame ecefFrame = measurement.getFrame();
        final ECEFFrame previousEcefFrame = measurement.getPreviousFrame();
        final double timeInterval = measurement.getTimeInterval();

        final BodyKinematics expectedKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, ecefFrame,
                        previousEcefFrame);

        final double fMeasX1 = measuredKinematics.getFx();
        final double fMeasY1 = measuredKinematics.getFy();
        final double fMeasZ1 = measuredKinematics.getFz();

        final double fTrueX = expectedKinematics.getFx();
        final double fTrueY = expectedKinematics.getFy();
        final double fTrueZ = expectedKinematics.getFz();

        final double[] b = preliminaryResult.mEstimatedBiases;
        final double bx = b[0];
        final double by = b[1];
        final double bz = b[2];

        final Matrix ma = preliminaryResult.mEstimatedMa;

        try {
            final Matrix m = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            m.add(ma);

            final Matrix ftrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            ftrue.setElementAtIndex(0, fTrueX);
            ftrue.setElementAtIndex(1, fTrueY);
            ftrue.setElementAtIndex(2, fTrueZ);

            m.multiply(ftrue);

            final double fMeasX2 = bx + m.getElementAtIndex(0);
            final double fMeasY2 = by + m.getElementAtIndex(1);
            final double fMeasZ2 = bz + m.getElementAtIndex(2);

            final double diffX = fMeasX2 - fMeasX1;
            final double diffY = fMeasY2 - fMeasY1;
            final double diffZ = fMeasZ2 - fMeasZ1;

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
    protected void computePreliminarySolutions(final int[] samplesIndices,
                                               final List<PreliminaryResult> solutions) {

        final List<StandardDeviationFrameBodyKinematics> measurements = new ArrayList<>();

        for (final int samplesIndex : samplesIndices) {
            measurements.add(mMeasurements.get(samplesIndex));
        }

        try {
            final PreliminaryResult result = new PreliminaryResult();
            result.mEstimatedBiases = getInitialBias();
            result.mEstimatedMa = getInitialMa();

            if (mUseLinearCalibrator) {
                mLinearCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mLinearCalibrator.setMeasurements(measurements);
                mLinearCalibrator.calibrate();

                mLinearCalibrator.getEstimatedBiases(result.mEstimatedBiases);
                result.mEstimatedMa = mLinearCalibrator.getEstimatedMa();
            }

            if (mRefinePreliminarySolutions) {
                mNonLinearCalibrator.setInitialBias(result.mEstimatedBiases);
                mNonLinearCalibrator.setInitialMa(result.mEstimatedMa);
                mNonLinearCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mNonLinearCalibrator.setMeasurements(measurements);
                mNonLinearCalibrator.calibrate();

                mNonLinearCalibrator.getEstimatedBiases(result.mEstimatedBiases);
                result.mEstimatedMa = mNonLinearCalibrator.getEstimatedMa();
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
                mNonLinearCalibrator.setInitialBias(preliminaryResult.mEstimatedBiases);
                mNonLinearCalibrator.setInitialMa(preliminaryResult.mEstimatedMa);
                mNonLinearCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mNonLinearCalibrator.setMeasurements(inlierMeasurements);
                mNonLinearCalibrator.calibrate();

                mEstimatedBiases = mNonLinearCalibrator.getEstimatedBiases();
                mEstimatedMa = mNonLinearCalibrator.getEstimatedMa();

                if (mKeepCovariance) {
                    mEstimatedCovariance = mNonLinearCalibrator.getEstimatedCovariance();
                } else {
                    mEstimatedCovariance = null;
                }

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                mEstimatedCovariance = null;
                mEstimatedBiases = preliminaryResult.mEstimatedBiases;
                mEstimatedMa = preliminaryResult.mEstimatedMa;
            }
        } else {
            mEstimatedCovariance = null;
            mEstimatedBiases = preliminaryResult.mEstimatedBiases;
            mEstimatedMa = preliminaryResult.mEstimatedMa;
        }
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
         * Estimated accelerometer biases for each IMU axis expressed in meter per squared
         * second (m/s^2).
         */
        private double[] mEstimatedBiases;

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
