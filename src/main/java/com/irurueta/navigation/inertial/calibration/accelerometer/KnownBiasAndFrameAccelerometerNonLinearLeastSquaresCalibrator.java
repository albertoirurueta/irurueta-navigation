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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.Collection;

/**
 * Estimates accelerometer cross couplings and scaling factors.
 * This estimator assumes that biases are known.
 * <p>
 * This calibrator uses an iterative approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator it is assumed that body position and orientation is known and
 * that body remains static on the same position while being calibrated.
 * <p>
 * Measured specific force is assumed to follow the model shown below:
 * <pre>
 *     fmeas = ba + (I + Ma) * ftrue + w
 * </pre>
 * Where:
 * - fmeas is the measured specific force. This is a 3x1 vector.
 * - ba is accelerometer bias. This is a known 3x1 vector.
 * - I is the 3x3 identity matrix.
 * - Ma is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect accelerometer, this should be a 3x3 zero matrix.
 * - ftrue is ground-trush specific force.
 * - w is measurement noise.
 */
public class KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator implements
        KnownBiasAndFrameAccelerometerCalibrator<StandardDeviationFrameBodyKinematics,
                KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener>,
        AccelerometerNonLinearCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 3;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 6;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 9;

    /**
     * Levenberg-Marquardt  fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter mFitter =
            new LevenbergMarquardtMultiVariateFitter();

    /**
     * Known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     */
    private double mBiasX;

    /**
     * Known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     */
    private double mBiasY;

    /**
     * Known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
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
     * Contains a collections of body kinematics measurements taken at different
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
    private Collection<? extends StandardDeviationFrameBodyKinematics> mMeasurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this estimator.
     */
    private KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener mListener;

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
     * Indicates whether estimator is running.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasY known y coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasZ known z coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasY    known y coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasZ    known z coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        this(biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        this(measurements, biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @param biasY known y coordinate of accelerometer bias.
     * @param biasZ known z coordinate of accelerometer bias.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of accelerometer bias.
     * @param biasY    known y coordinate of accelerometer bias.
     * @param biasZ    known z coordinate of accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final boolean commonAxisUsed) {
        this(biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final boolean commonAxisUsed) {
        this(measurements, biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     known x coordinate of accelerometer bias.
     * @param biasY     known y coordinate of accelerometer bias.
     * @param biasZ     known z coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
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
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     known x coordinate of accelerometer bias.
     * @param biasY     known y coordinate of accelerometer bias.
     * @param biasZ     known z coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx, final double initialSy,
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
     * @param biasX     known x coordinate of accelerometer bias.
     * @param biasY     known y coordinate of accelerometer bias.
     * @param biasZ     known z coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz) {
        this(biasX, biasY, biasZ, commonAxisUsed, initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed, initialSx, initialSy,
                initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      known x coordinate of accelerometer bias expressed in meters per
     *                   squared second (m/s^2).
     * @param biasY      known y coordinate of accelerometer bias expressed in meters per
     *                   squared second (m/s^2).
     * @param biasZ      known z coordinate of accelerometer bias expressed in meters per
     *                   squared second (m/s^2).
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
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
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
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
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, commonAxisUsed, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed, initialSx, initialSy,
                initialSz, initialMxy, initialMxz, initialMyx, initialMyz, initialMzx,
                initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      known x coordinate of accelerometer bias.
     * @param biasY      known y coordinate of accelerometer bias.
     * @param biasZ      known z coordinate of accelerometer bias.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
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
     * @param biasX      known x coordinate of accelerometer bias.
     * @param biasY      known y coordinate of accelerometer bias.
     * @param biasZ      known z coordinate of accelerometer bias.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
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
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, commonAxisUsed, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
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
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed, initialSx, initialSy,
                initialSz, initialMxy, initialMxz, initialMyx, initialMyz, initialMzx,
                initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known bias. This must have length 3 and is expressed in meters per
     *             squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
     * @param bias     known bias. This must have length 3 and is expressed in meters per
     *                 squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double[] bias,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias. This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias) {
        this(bias);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias. This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias           known bias. This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double[] bias, final boolean commonAxisUsed) {
        this(bias);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param bias           known bias. This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double[] bias, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(bias, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known bias. This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed) {
        this(bias, commonAxisUsed);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known bias. This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known bias. This must be a 3x1 matrix expressed in meters per
     *             squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix bias) {
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias     known bias. This must be a 3x1 matrix expressed in meters per
     *                 squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix bias,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias. This must be a 3x1 matrix expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias) {
        this(bias);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias. This must be a 3x1 matrix expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias           known bias. This must be a 3x1 matrix expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix bias, final boolean commonAxisUsed) {
        this(bias);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param bias           known bias. This must be a 3x1 matrix expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix bias, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(bias, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known bias. This must be a 3x1 matrix expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed) {
        this(bias, commonAxisUsed);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known bias. This must be a 3x1 matrix expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias      known bias. This must be a 3x1 matrix expressed in meters per
     *                  squared second (m/s^2).
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
     * @param bias      known bias. This must be a 3x1 matrix expressed in meters per
     *                  squared second (m/s^2).
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(bias, initialMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias. This must be a 3x1 matrix expressed in meters per
     *                     squared second (m/s^2).
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(bias, initialMa);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias. This must be a 3x1 matrix expressed in meters per
     *                     squared second (m/s^2).
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias, initialMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias           known bias. This must be a 3x1 matrix expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix bias, final boolean commonAxisUsed,
            final Matrix initialMa) {
        this(bias, initialMa);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param bias           known bias. This must be a 3x1 matrix expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix bias, final boolean commonAxisUsed,
            final Matrix initialMa,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(bias, commonAxisUsed, initialMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known bias. This must be a 3x1 matrix expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed, final Matrix initialMa) {
        this(bias, commonAxisUsed, initialMa);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known bias. This must be a 3x1 matrix expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed, final Matrix initialMa,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias, commonAxisUsed, initialMa);
        mListener = listener;
    }

    /**
     * Gets known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return known x coordinate of accelerometer bias.
     */
    @Override
    public double getBiasX() {
        return mBiasX;
    }

    /**
     * Sets known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final double biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
    }

    /**
     * Gets known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return known y coordinate of accelerometer bias.
     */
    @Override
    public double getBiasY() {
        return mBiasY;
    }

    /**
     * Sets known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasY known y coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final double biasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = biasY;
    }

    /**
     * Gets known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return known z coordinate of accelerometer bias.
     */
    @Override
    public double getBiasZ() {
        return mBiasZ;
    }

    /**
     * Sets known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasZ known z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final double biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = biasZ;
    }

    /**
     * Gets known x coordinate of accelerometer bias.
     *
     * @return known x coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasXAsAcceleration() {
        return new Acceleration(mBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known x coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasXAsAcceleration(final Acceleration result) {
        result.setValue(mBiasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known x coordinate of accelerometer bias.
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final Acceleration biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = convertAcceleration(biasX);
    }

    /**
     * Gets known y coordinate of accelerometer bias.
     *
     * @return known y coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasYAsAcceleration() {
        return new Acceleration(mBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known y coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasYAsAcceleration(final Acceleration result) {
        result.setValue(mBiasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known y coordinate of accelerometer bias.
     *
     * @param biasY known y coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final Acceleration biasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = convertAcceleration(biasY);
    }

    /**
     * Gets known z coordinate of accelerometer bias.
     *
     * @return known z coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasZAsAcceleration() {
        return new Acceleration(mBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known z coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasZAsAcceleration(final Acceleration result) {
        result.setValue(mBiasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known z coordinate of accelerometer bias.
     *
     * @param biasZ known z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final Acceleration biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = convertAcceleration(biasZ);
    }

    /**
     * Sets known accelerometer bias coordinates expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @param biasY known y coordinate of accelerometer bias.
     * @param biasZ known z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(final double biasX, final double biasY, final double biasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasX = biasX;
        mBiasY = biasY;
        mBiasZ = biasZ;
    }

    /**
     * Sets known accelerometer bias coordinates.
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @param biasY known y coordinate of accelerometer bias.
     * @param biasZ known z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(final Acceleration biasX, final Acceleration biasY,
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
    @Override
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
    @Override
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
    @Override
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
    @Override
    public double getInitialSx() {
        return mInitialSx;
    }

    /**
     * Sets initial x scaling factor.
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
     *
     * @return initial y scaling factor.
     */
    @Override
    public double getInitialSy() {
        return mInitialSy;
    }

    /**
     * Sets initial y scaling factor.
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
     *
     * @return initial z scaling factor.
     */
    @Override
    public double getInitialSz() {
        return mInitialSz;
    }

    /**
     * Sets initial z scaling factor.
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
     *
     * @return initial x-y cross coupling error.
     */
    @Override
    public double getInitialMxy() {
        return mInitialMxy;
    }

    /**
     * Sets initial x-y cross coupling error.
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
     *
     * @return initial x-z cross coupling error.
     */
    @Override
    public double getInitialMxz() {
        return mInitialMxz;
    }

    /**
     * Sets initial x-z cross coupling error.
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
     *
     * @return initial y-x cross coupling error.
     */
    @Override
    public double getInitialMyx() {
        return mInitialMyx;
    }

    /**
     * Sets initial y-x cross coupling error.
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
     *
     * @return initial y-z cross coupling error.
     */
    @Override
    public double getInitialMyz() {
        return mInitialMyz;
    }

    /**
     * Sets initial y-z cross coupling error.
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
     *
     * @return initial z-x cross coupling error.
     */
    @Override
    public double getInitialMzx() {
        return mInitialMzx;
    }

    /**
     * Sets initial z-x cross coupling error.
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
     *
     * @return initial z-y cross coupling error.
     */
    @Override
    public double getInitialMzy() {
        return mInitialMzy;
    }

    /**
     * Sets initial z-y cross coupling error.
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
     * Gets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @return array containint coordinates of known bias.
     */
    @Override
    public double[] getBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
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
     * Sets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param bias known accelerometer bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
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
     * Gets known accelerometer bias as a column matrix.
     *
     * @return known accelerometer bias as a column matrix.
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
     * Gets known accelerometer bias as a column matrix.
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
     * Sets known accelerometer bias as a column matrix.
     *
     * @param bias accelerometer bias to be set.
     * @throws LockedException          if calibrator is currently running
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
     * Gets initial scale factors and cross coupling errors matrix.
     *
     * @return initial scale factors and cross coupling errors matrix.
     */
    @Override
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
    @Override
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
    @Override
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
     * Gets a collection of body kinematics measurements taken at different
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
    @Override
    public Collection<? extends StandardDeviationFrameBodyKinematics> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets a collection of body kinematics measurements taken at different
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
    @Override
    public void setMeasurements(
            final Collection<? extends StandardDeviationFrameBodyKinematics> measurements)
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
    @Override
    public KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setListener(
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener)
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
     * Estimates accelerometer calibration parameters containing bias, scale factors
     * and cross-coupling errors.
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

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onCalibrateStart(this);
            }

            if (mCommonAxisUsed) {
                calibrateCommonAxis();
            } else {
                calibrateGeneral();
            }

            if (mListener != null) {
                mListener.onCalibrateEnd(this);
            }

        } catch (final AlgebraException | FittingException |
                com.irurueta.numerical.NotReadyException e) {
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
    @Override
    public Matrix getEstimatedMa() {
        return mEstimatedMa;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return mEstimatedMa != null ?
                mEstimatedMa.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    @Override
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
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException                         if there are numerical errors.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // Hence:
        //  [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        //  [fmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        //  [fmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

        // where myx = mzx = mzy = 0

        // Hence:
        //  [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        //  [fmeasy] = [by]     [0  1   0]   [0     sy  myz]    [ftruey]
        //  [fmeasz] = [bz]     [0  0   1]   [0     0   sz ]    [ftruez]

        //  [fmeasx] = [bx] +   [1+sx   mxy     mxz ][ftruex]
        //  [fmeasy]   [by]     [0      1+sy    myz ][ftruey]
        //  [fmeasz]   [bz]     [0      0       1+sz][ftruez]

        //  fmeasx = bx + (1+sx) * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + (1+sy) * ftruey + myz * ftruez
        //  fmeasz = bz + (1+sz) * ftruez

        // Where the unknowns are: sx, sy, sz, mxy mxz, myz
        // Reordering:
        //  fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + ftruey + sy * ftruey + myz * ftruez
        //  fmeasz = bz + ftruez + sz * ftruez

        //  fmeasx - ftruex - bx = sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy - ftruey - by = sy * ftruey + myz * ftruez
        //  fmeasz - ftruez - bz = sz * ftruez

        // [ftruex  0       0       ftruey  ftruez  0     ][sx ] = [fmeasx - ftruex - bx]
        // [0       ftruey  0       0       0       ftruez][sy ]   [fmeasy - ftruey - by]
        // [0       0       ftruez  0       0       0     ][sz ]   [fmeasz - ftruez - bz]
        //                                                 [mxy]
        //                                                 [mxz]
        //                                                 [myz]

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true specific force coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured specific force
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                initial[0] = mInitialSx;
                initial[1] = mInitialSy;
                initial[2] = mInitialSz;

                initial[3] = mInitialMxy;
                initial[4] = mInitialMxz;
                initial[5] = mInitialMyz;

                return initial;
            }

            @Override
            public void evaluate(
                    final int i, final double[] point, final double[] result,
                    final double[] params, final Matrix jacobian) {
                // We know that
                //  fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
                //  fmeasy = by + ftruey + sy * ftruey + myz * ftruez
                //  fmeasz = bz + ftruez + sz * ftruez

                // Hence, the derivatives respect the parameters sx, sy, sz, mxy, mxz, myz are:

                // d(fmeasx)/d(sx) = ftruex
                // d(fmeasx)/d(sy) = 0.0
                // d(fmeasx)/d(sz) = 0.0
                // d(fmeasx)/d(mxy) = ftruey
                // d(fmeasx)/d(mxz) = ftruez
                // d(fmeasx)/d(myz) = 0.0

                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = ftruey
                // d(fmeasy)/d(sz) = 0.0
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myz) = ftruez

                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = 0.0
                // d(fmeasy)/d(sz) = ftruez
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myz) = 0.0

                final double sx = params[0];
                final double sy = params[1];
                final double sz = params[2];

                final double mxy = params[3];
                final double mxz = params[4];
                final double myz = params[5];

                final double ftruex = point[0];
                final double ftruey = point[1];
                final double ftruez = point[2];

                result[0] = mBiasX + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez;
                result[1] = mBiasY + ftruey + sy * ftruey + myz * ftruez;
                result[2] = mBiasZ + ftruez + sz * ftruez;

                jacobian.setElementAt(0, 0, ftruex);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, ftruey);
                jacobian.setElementAt(0, 4, ftruez);
                jacobian.setElementAt(0, 5, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, ftruey);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, 0.0);
                jacobian.setElementAt(1, 5, ftruez);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, ftruez);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, 0.0);
            }
        });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double sx = result[0];
        final double sy = result[1];
        final double sz = result[2];

        final double mxy = result[3];
        final double mxz = result[4];
        final double myz = result[5];

        if (mEstimatedMa == null) {
            mEstimatedMa = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } else {
            mEstimatedMa.initialize(0.0);
        }

        mEstimatedMa.setElementAt(0, 0, sx);

        mEstimatedMa.setElementAt(0, 1, mxy);
        mEstimatedMa.setElementAt(1, 1, sy);

        mEstimatedMa.setElementAt(0, 2, mxz);
        mEstimatedMa.setElementAt(1, 2, myz);
        mEstimatedMa.setElementAt(2, 2, sz);

        mEstimatedCovariance = mFitter.getCovar();

        // propagate covariance matrix so that all parameters are taken into
        // account in the order: sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy

        // We define a lineal function mapping original parameters for the common
        // axis case to the general case
        //[sx'] = [1  0  0  0  0  0][sx]
        //[sy']   [0  1  0  0  0  0][sy]
        //[sz']   [0  0  1  0  0  0][sz]
        //[mxy']  [0  0  0  1  0  0][mxy]
        //[mxz']  [0  0  0  0  1  0][mxz]
        //[myx']  [0  0  0  0  0  0][myz]
        //[myz']  [0  0  0  0  0  1]
        //[mzx']  [0  0  0  0  0  0]
        //[mzy']  [0  0  0  0  0  0]

        // As defined in com.irurueta.statistics.MultivariateNormalDist,
        // if we consider the jacobian of the lineal application the matrix shown
        // above, then covariance can be propagated as follows
        final Matrix jacobian = Matrix.identity(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);
        jacobian.setElementAt(5, 5, 0.0);
        jacobian.setElementAt(6, 5, 1.0);

        // propagated covariance is J * Cov * J'
        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
        mEstimatedChiSq = mFitter.getChisq();
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws AlgebraException                         if there are numerical errors.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateGeneral() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // Hence:
        //  [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        //  [fmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        //  [fmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

        //  [fmeasx] = [bx] +   [1+sx   mxy     mxz ][ftruex]
        //  [fmeasy]   [by]     [myx    1+sy    myz ][ftruey]
        //  [fmeasz]   [bz]     [mzx    mzy     1+sz][ftruez]

        //  fmeasx = bx + (1+sx) * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + myx * ftruex + (1+sy) * ftruey + myz * ftruez
        //  fmeasz = bz + mzx * ftruex + mzy * ftruey + (1+sz) * ftruez

        // Where the unknowns are: sx, sy, sz, mxy mxz, myx, myz, mzx, mzy
        // Reordering:
        //  fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
        //  fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

        //  fmeasx - ftruex - bx = sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy - ftruey - by = myx * ftruex + sy * ftruey + myz * ftruez
        //  fmeasz - ftruez - bz = mzx * ftruex + mzy * ftruey + sz * ftruez

        // [ftruex  0       0       ftruey  ftruez  0       0       0       0     ][sx ] = [fmeasx - ftruex - bx]
        // [0       ftruey  0       0       0       ftruex  ftruez  0       0     ][sy ]   [fmeasy - ftruey - by]
        // [0       0       ftruez  0       0       0       0       ftruex  ftruey][sz ]   [fmeasz - ftruez - bz]
        //                                                                         [mxy]
        //                                                                         [mxz]
        //                                                                         [myx]
        //                                                                         [myz]
        //                                                                         [mzx]
        //                                                                         [mzy]

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true specific force coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured specific force
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[GENERAL_UNKNOWNS];

                initial[0] = mInitialSx;
                initial[1] = mInitialSy;
                initial[2] = mInitialSz;

                initial[3] = mInitialMxy;
                initial[4] = mInitialMxz;
                initial[5] = mInitialMyx;
                initial[6] = mInitialMyz;
                initial[7] = mInitialMzx;
                initial[8] = mInitialMzy;

                return initial;

            }

            @Override
            public void evaluate(
                    final int i, final double[] point, final double[] result,
                    final double[] params, final Matrix jacobian) {
                // We know that:
                //  fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
                //  fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
                //  fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

                // Hence, the derivatives respect the parameters sx, sy, sz,
                // mxy, mxz, myx, myz, mzx and mzy is:

                // d(fmeasx)/d(sx) = ftruex
                // d(fmeasx)/d(sy) = 0.0
                // d(fmeasx)/d(sz) = 0.0
                // d(fmeasx)/d(mxy) = ftruey
                // d(fmeasx)/d(mxz) = ftruez
                // d(fmeasx)/d(myx) = 0.0
                // d(fmeasx)/d(myz) = 0.0
                // d(fmeasx)/d(mzx) = 0.0
                // d(fmeasx)/d(mzy) = 0.0

                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = ftruey
                // d(fmeasy)/d(sz) = 0.0
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myx) = ftruex
                // d(fmeasy)/d(myz) = ftruez
                // d(fmeasy)/d(mzx) = 0.0
                // d(fmeasy)/d(mzy) = 0.0

                // d(fmeasz)/d(sx) = 0.0
                // d(fmeasz)/d(sy) = 0.0
                // d(fmeasz)/d(sz) = ftruez
                // d(fmeasz)/d(mxy) = 0.0
                // d(fmeasz)/d(mxz) = 0.0
                // d(fmeasz)/d(myx) = 0.0
                // d(fmeasz)/d(myz) = 0.0
                // d(fmeasz)/d(mzx) = ftruex
                // d(fmeasz)/d(mzy) = ftruey

                final double sx = params[0];
                final double sy = params[1];
                final double sz = params[2];

                final double mxy = params[3];
                final double mxz = params[4];
                final double myx = params[5];
                final double myz = params[6];
                final double mzx = params[7];
                final double mzy = params[8];

                final double ftruex = point[0];
                final double ftruey = point[1];
                final double ftruez = point[2];

                result[0] = mBiasX + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez;
                result[1] = mBiasY + myx * ftruex + ftruey + sy * ftruey + myz * ftruez;
                result[2] = mBiasZ + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez;

                jacobian.setElementAt(0, 0, ftruex);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, ftruey);
                jacobian.setElementAt(0, 4, ftruez);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, 0.0);
                jacobian.setElementAt(0, 7, 0.0);
                jacobian.setElementAt(0, 8, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, ftruey);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, 0.0);
                jacobian.setElementAt(1, 5, ftruex);
                jacobian.setElementAt(1, 6, ftruez);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, 0.0);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, ftruez);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, 0.0);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, ftruex);
                jacobian.setElementAt(2, 8, ftruey);
            }
        });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double sx = result[0];
        final double sy = result[1];
        final double sz = result[2];

        final double mxy = result[3];
        final double mxz = result[4];
        final double myx = result[5];
        final double myz = result[6];
        final double mzx = result[7];
        final double mzy = result[8];

        if (mEstimatedMa == null) {
            mEstimatedMa = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } else {
            mEstimatedMa.initialize(0.0);
        }

        mEstimatedMa.setElementAt(0, 0, sx);
        mEstimatedMa.setElementAt(1, 0, myx);
        mEstimatedMa.setElementAt(2, 0, mzx);

        mEstimatedMa.setElementAt(0, 1, mxy);
        mEstimatedMa.setElementAt(1, 1, sy);
        mEstimatedMa.setElementAt(2, 1, mzy);

        mEstimatedMa.setElementAt(0, 2, mxz);
        mEstimatedMa.setElementAt(1, 2, myz);
        mEstimatedMa.setElementAt(2, 2, sz);

        mEstimatedCovariance = mFitter.getCovar();
        mEstimatedChiSq = mFitter.getChisq();
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     */
    private void setInputData() throws WrongSizeException {
        // set input data using:
        //  fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
        //  fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

        final BodyKinematics expectedKinematics = new BodyKinematics();

        final int numMeasurements = mMeasurements.size();
        final Matrix x = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final Matrix y = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final double[] specificForceStandardDeviations = new double[numMeasurements];
        int i = 0;
        for (final StandardDeviationFrameBodyKinematics measurement : mMeasurements) {
            final BodyKinematics measuredKinematics = measurement.getKinematics();
            final ECEFFrame ecefFrame = measurement.getFrame();
            final ECEFFrame previousEcefFrame = measurement.getPreviousFrame();
            final double timeInterval = measurement.getTimeInterval();

            ECEFKinematicsEstimator.estimateKinematics(timeInterval, ecefFrame,
                    previousEcefFrame, expectedKinematics);

            final double fMeasX = measuredKinematics.getFx();
            final double fMeasY = measuredKinematics.getFy();
            final double fMeasZ = measuredKinematics.getFz();

            final double fTrueX = expectedKinematics.getFx();
            final double fTrueY = expectedKinematics.getFy();
            final double fTrueZ = expectedKinematics.getFz();

            x.setElementAt(i, 0, fTrueX);
            x.setElementAt(i, 1, fTrueY);
            x.setElementAt(i, 2, fTrueZ);

            y.setElementAt(i, 0, fMeasX);
            y.setElementAt(i, 1, fMeasY);
            y.setElementAt(i, 2, fMeasZ);

            specificForceStandardDeviations[i] =
                    measurement.getSpecificForceStandardDeviation();
            i++;
        }

        mFitter.setInputData(x, y, specificForceStandardDeviations);
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
}