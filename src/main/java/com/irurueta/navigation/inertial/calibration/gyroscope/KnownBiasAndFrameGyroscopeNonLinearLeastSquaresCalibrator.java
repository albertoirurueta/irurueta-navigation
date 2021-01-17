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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.Collection;

/**
 * Estimates gyroscope cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * <p>
 * This calibrator uses an iterative approach to find a minimum least squared error
 * solution.
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
public class KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator implements
        KnownBiasAndFrameGyroscopeCalibrator<StandardDeviationFrameBodyKinematics,
                KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener>,
        GyroscopeNonLinearCalibrator {

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
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 15;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 18;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter mFitter =
            new LevenbergMarquardtMultiVariateFitter();

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
     * Listener to handle events raised by this calibrator.
     */
    private KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener mListener;

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
     * Indicates whether calibrator is running.
     */
    private boolean mRunning;

    /**
     * Constructor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator() {
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
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this();
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements) {
        this();
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     * @param biasY known y coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     * @param biasZ known z coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        this();
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param biasY    known y coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param biasZ    known z coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ) {
        this(biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias.
     * @param biasY known y coordinate of gyroscope bias.
     * @param biasZ known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ) {
        this();
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias.
     * @param biasY    known y coordinate of gyroscope bias.
     * @param biasZ    known z coordinate of gyroscope bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ) {
        this(biasX, biasY, biasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ) {
        this(biasX, biasY, biasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     known x coordinate of gyroscope bias expressed in radians per second
     *                  (rad/s).
     * @param biasY     known y coordinate of gyroscope bias expressed in radians per second
     *                  (rad/s).
     * @param biasZ     known z coordinate of gyroscope bias expressed in radians per second
     *                  (rad/s).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     known x coordinate of gyroscope bias.
     * @param biasY     known y coordinate of gyroscope bias.
     * @param biasZ     known z coordinate of gyroscope bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final double initialSx, final double initialSy,
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
     * @param biasX     known x coordinate of gyroscope bias.
     * @param biasY     known y coordinate of gyroscope bias.
     * @param biasZ     known z coordinate of gyroscope bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
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
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
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
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed biasX,
            final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      known x coordinate of gyroscope bias expressed in radians per second
     *                   (rad/s).
     * @param biasY      known y coordinate of gyroscope bias expressed in radians per second
     *                   (rad/s).
     * @param biasZ      known z coordinate of gyroscope bias expressed in radians per second
     *                   (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
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
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
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
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      known x coordinate of gyroscope bias.
     * @param biasY      known y coordinate of gyroscope bias.
     * @param biasZ      known z coordinate of gyroscope bias.
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
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
     * @param biasX      known x coordinate of gyroscope bias.
     * @param biasY      known y coordinate of gyroscope bias.
     * @param biasZ      known z coordinate of gyroscope bias.
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
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
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
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
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known gyroscope bias expressed in radians per second (rad/s).
     *             This must have length 3.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double[] bias) {
        this();
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias     known gyroscope bias expressed in radians per second (rad/s).
     *                 This must have length 3.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double[] bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias expressed in radians per second (rad/s).
     *                     This must have length 3.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     * @param bias         known gyroscope bias expressed in radians per second (rad/s).
     *                     This must have length 3.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known gyroscope bias expressed in radians per second (rad/s).
     *                       This must have length 3.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] bias) {
        this(bias);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known gyroscope bias expressed in radians per second (rad/s).
     *                       This must have length 3.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, bias);
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
     * @param bias           known gyroscope bias expressed in radians per second (rad/s).
     *                       This must have length 3.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(commonAxisUsed, bias);
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
     * @param bias           known gyroscope bias expressed in radians per second (rad/s).
     *                       This must have length 3.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known bias expressed in radians per second (rad/s). This must
     *             be 3x1.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias) {
        this();
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias     known bias expressed in radians per second (rad/s). This must
     *                 be 3x1.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias) {
        this(bias);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, bias);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(commonAxisUsed, bias);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, bias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias      known bias expressed in radians per second (rad/s). This must
     *                  be 3x1.
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias, final Matrix initialMg) {
        this(bias);
        try {
            setInitialMg(initialMg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias      known bias expressed in radians per second (rad/s). This must
     *                  be 3x1.
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias, final Matrix initialMg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(bias, initialMg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMg) {
        this(bias, initialMg);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias, initialMg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMg) {
        this(bias, initialMg);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, bias, initialMg);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMg) {
        this(commonAxisUsed, bias, initialMg);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, bias, initialMg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias      known bias expressed in radians per second (rad/s). This must
     *                  be 3x1.
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @param initialGg initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias, final Matrix initialMg,
            final Matrix initialGg) {
        this(bias, initialMg);
        try {
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias      known bias expressed in radians per second (rad/s). This must
     *                  be 3x1.
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @param initialGg initial G-dependent cross biases.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias, final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(bias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param initialGg    initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMg,
            final Matrix initialGg) {
        this(bias, initialMg, initialGg);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param initialGg    initial G-dependent cross biases.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg,
            final Matrix initialGg) {
        this(bias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg,
            final Matrix initialGg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, bias, initialMg, initialGg);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg,
            final Matrix initialGg) {
        this(commonAxisUsed, bias, initialMg, initialGg);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, bias, initialMg, initialGg);
        mListener = listener;
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
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) throws LockedException {
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
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) throws LockedException {
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
     * Gets a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate a given IMU model (e.g. obtain
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
     * frames (positions, orientations and velocities).
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
     * When enabled, this eliminates 3 variables from Mg matrix.
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
     * When enabled, this eliminates 3 variables from Mg matrix.
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
     * Gets listener to handle events raised by this calibrator.
     *
     * @return listener to handle events raised by this calibrator.
     */
    @Override
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setListener(
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
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
     * Sets known x coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
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
     * (rad/s).
     *
     * @param biasY y coordinate of gyroscope bias.
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
     * Gets known x coordinate of gyroscope bias.
     *
     * @return x coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedX() {
        return new AngularSpeed(mBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known x coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(mBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known x coordinate of gyroscope bias.
     *
     * @param biasX x coordinate of gyroscope bias.
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
     * Gets known y coordinate of gyroscope bias.
     *
     * @return y coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedY() {
        return new AngularSpeed(mBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known y coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(mBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known y coordinate of gyroscope bias.
     *
     * @param biasY y coordinate of gyroscope bias.
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
     * Gets known z coordinate of gyroscope bias.
     *
     * @return z coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedZ() {
        return new AngularSpeed(mBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known z coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known z coordinate of gyroscope bias.
     *
     * @param biasZ z coordinate of gyroscope bias.
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
     * Sets known gyroscope bias coordinates expressed in radians per second
     * (rad/s).
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

        mBiasX = biasX;
        mBiasY = biasY;
        mBiasZ = biasZ;
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
    public void setBiasCoordinates(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ)
            throws LockedException {
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
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinate of known bias.
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
     * Indicates whether calibrator is ready to start the calibration.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mMeasurements != null
                && mMeasurements.size() >= MINIMUM_MEASUREMENTS;
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
     * Estimates gyroscope calibration parameters containing bias, scale factors,
     * cross-coupling errors and g-dependant cross biases.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if calibration fails for numerical reasons.
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

        } catch (final AlgebraException | FittingException
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
     * @return estimated gyroscope scale factors and cross coupling errors.
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
     *
     * @return a 3x3 matrix containing g-dependent cross biases.
     */
    @Override
    public Matrix getEstimatedGg() {
        return mEstimatedGg;
    }

    /**
     * Gets estimated covariance matrix for estimated position.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32, gg13, gg23, gg33.
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
    @Override
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
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException                         if there are numerical errors.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {

        // The gyroscope model is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // where myx = mzx = mzy = 0

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [0     sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [0     0      sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        // [Ωmeasx] = [bx] + ( [1+sx  mxy    mxz ]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0     1+sy   myz ]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0     0      1+sz]  [Ωtruez]   [g31   g32   g33][ftruez]

        // Ωmeasx = bx + (1+sx) * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + (1+sy) * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + (1+sz) * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Where the unknowns are: sx, sy, sz, mxy mxz, myz, g11, g12, g13, g21, g22, g23, g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex - bx = sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey - by = sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez - bz = sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [Ωtruex  0       0       Ωtruey  Ωtruez  0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][sx ] =  [Ωmeasx - Ωtruex - bx]
        // [0       Ωtruey  0       0       0       Ωtruez  0       0       0       ftruex  ftruey  ftruez  0       0       0     ][sy ]    [Ωmeasy - Ωtruey - by]
        // [0       0       Ωtruez  0       0       0       0       0       0       0       0       0       ftruex  ftruey  ftruez][sz ]    [Ωmeasz - Ωtruez - bz]
        //                                                                                                                         [mxy]
        //                                                                                                                         [mxz]
        //                                                                                                                         [myz]
        //                                                                                                                         [g11]
        //                                                                                                                         [g12]
        //                                                                                                                         [g13]
        //                                                                                                                         [g21]
        //                                                                                                                         [g22]
        //                                                                                                                         [g23]
        //                                                                                                                         [g31]
        //                                                                                                                         [g32]
        //                                                                                                                         [g33]

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiVariateFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Input points are true angular rate + specific force coordinates
                        return 2 * BodyKinematics.COMPONENTS;
                    }

                    @Override
                    public int getNumberOfVariables() {
                        // The multivariate function returns the components of measured angular rate
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

                        final double[] buffer = mInitialGg.getBuffer();
                        final int num = buffer.length;
                        System.arraycopy(buffer, 0, initial, 6, num);

                        return initial;
                    }

                    @Override
                    public void evaluate(final int i, final double[] point,
                                         final double[] result, final double[] params,
                                         final Matrix jacobian) {
                        // We know that:
                        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
                        // Ωmeasy = by + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
                        // Ωmeasz = bz + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

                        // Hence, the derivatives respect the parameters
                        // sx, sy, sz, mxy mxz, myz, g11, g12, g13, g21, g22, g23,
                        // g31, g32, and g33 is:

                        // d(Ωmeasx)/d(sx) = Ωtruex
                        // d(Ωmeasx)/d(sy) = 0.0
                        // d(Ωmeasx)/d(sz) = 0.0
                        // d(Ωmeasx)/d(mxy) = Ωtruey
                        // d(Ωmeasx)/d(mxz) = Ωtruez
                        // d(Ωmeasx)/d(myz) = 0.0
                        // d(Ωmeasx)/d(g11) = ftruex
                        // d(Ωmeasx)/d(g12) = ftruey
                        // d(Ωmeasx)/d(g13) = ftruez
                        // d(Ωmeasx)/d(g21) = 0.0
                        // d(Ωmeasx)/d(g22) = 0.0
                        // d(Ωmeasx)/d(g23) = 0.0
                        // d(Ωmeasx)/d(g31) = 0.0
                        // d(Ωmeasx)/d(g32) = 0.0
                        // d(Ωmeasx)/d(g33) = 0.0

                        // d(Ωmeasy)/d(sx) = 0.0
                        // d(Ωmeasy)/d(sy) = Ωtruey
                        // d(Ωmeasy)/d(sz) = 0.0
                        // d(Ωmeasy)/d(mxy) = 0.0
                        // d(Ωmeasy)/d(mxz) = 0.0
                        // d(Ωmeasy)/d(myz) = Ωtruez
                        // d(Ωmeasx)/d(g11) = 0.0
                        // d(Ωmeasx)/d(g12) = 0.0
                        // d(Ωmeasx)/d(g13) = 0.0
                        // d(Ωmeasx)/d(g21) = ftruex
                        // d(Ωmeasx)/d(g22) = ftruey
                        // d(Ωmeasx)/d(g23) = ftruez
                        // d(Ωmeasx)/d(g31) = 0.0
                        // d(Ωmeasx)/d(g32) = 0.0
                        // d(Ωmeasx)/d(g33) = 0.0

                        // d(Ωmeasz)/d(sx) = 0.0
                        // d(Ωmeasz)/d(sy) = 0.0
                        // d(Ωmeasz)/d(sz) = Ωtruez
                        // d(Ωmeasz)/d(mxy) = 0.0
                        // d(Ωmeasz)/d(mxz) = 0.0
                        // d(Ωmeasz)/d(myz) = 0.0
                        // d(Ωmeasx)/d(g11) = 0.0
                        // d(Ωmeasx)/d(g12) = 0.0
                        // d(Ωmeasx)/d(g13) = 0.0
                        // d(Ωmeasx)/d(g21) = 0.0
                        // d(Ωmeasx)/d(g22) = 0.0
                        // d(Ωmeasx)/d(g23) = 0.0
                        // d(Ωmeasx)/d(g31) = ftruex
                        // d(Ωmeasx)/d(g32) = ftruey
                        // d(Ωmeasx)/d(g33) = ftruez

                        final double sx = params[0];
                        final double sy = params[1];
                        final double sz = params[2];

                        final double mxy = params[3];
                        final double mxz = params[4];
                        final double myz = params[5];

                        final double g11 = params[6];
                        final double g21 = params[7];
                        final double g31 = params[8];
                        final double g12 = params[9];
                        final double g22 = params[10];
                        final double g32 = params[11];
                        final double g13 = params[12];
                        final double g23 = params[13];
                        final double g33 = params[14];

                        final double omegatruex = point[0];
                        final double omegatruey = point[1];
                        final double omegatruez = point[2];

                        final double ftruex = point[3];
                        final double ftruey = point[4];
                        final double ftruez = point[5];

                        result[0] = mBiasX + omegatruex + sx * omegatruex + mxy * omegatruey + mxz * omegatruez
                                + g11 * ftruex + g12 * ftruey + g13 * ftruez;
                        result[1] = mBiasY + omegatruey + sy * omegatruey + myz * omegatruez
                                + g21 * ftruex * g22 * ftruey + g23 * ftruez;
                        result[2] = mBiasZ + omegatruez + sz * omegatruez
                                + g31 * ftruex + g32 * ftruey + g33 * ftruez;

                        jacobian.setElementAt(0, 0, omegatruex);
                        jacobian.setElementAt(0, 1, 0.0);
                        jacobian.setElementAt(0, 2, 0.0);
                        jacobian.setElementAt(0, 3, omegatruey);
                        jacobian.setElementAt(0, 4, omegatruez);
                        jacobian.setElementAt(0, 5, 0.0);
                        jacobian.setElementAt(0, 6, ftruex);
                        jacobian.setElementAt(0, 7, ftruey);
                        jacobian.setElementAt(0, 8, ftruez);
                        jacobian.setElementAt(0, 9, 0.0);
                        jacobian.setElementAt(0, 10, 0.0);
                        jacobian.setElementAt(0, 11, 0.0);
                        jacobian.setElementAt(0, 12, 0.0);
                        jacobian.setElementAt(0, 13, 0.0);
                        jacobian.setElementAt(0, 14, 0.0);

                        jacobian.setElementAt(1, 0, 0.0);
                        jacobian.setElementAt(1, 1, omegatruey);
                        jacobian.setElementAt(1, 2, 0.0);
                        jacobian.setElementAt(1, 3, 0.0);
                        jacobian.setElementAt(1, 4, 0.0);
                        jacobian.setElementAt(1, 5, omegatruez);
                        jacobian.setElementAt(1, 6, 0.0);
                        jacobian.setElementAt(1, 7, 0.0);
                        jacobian.setElementAt(1, 8, 0.0);
                        jacobian.setElementAt(1, 9, ftruex);
                        jacobian.setElementAt(1, 10, ftruey);
                        jacobian.setElementAt(1, 11, ftruez);
                        jacobian.setElementAt(1, 12, 0.0);
                        jacobian.setElementAt(1, 13, 0.0);
                        jacobian.setElementAt(1, 14, 0.0);

                        jacobian.setElementAt(2, 0, 0.0);
                        jacobian.setElementAt(2, 1, 0.0);
                        jacobian.setElementAt(2, 2, omegatruez);
                        jacobian.setElementAt(2, 3, 0.0);
                        jacobian.setElementAt(2, 4, 0.0);
                        jacobian.setElementAt(2, 5, 0.0);
                        jacobian.setElementAt(2, 6, 0.0);
                        jacobian.setElementAt(2, 7, 0.0);
                        jacobian.setElementAt(2, 8, 0.0);
                        jacobian.setElementAt(2, 9, 0.0);
                        jacobian.setElementAt(2, 10, 0.0);
                        jacobian.setElementAt(2, 11, 0.0);
                        jacobian.setElementAt(2, 12, ftruex);
                        jacobian.setElementAt(2, 13, ftruey);
                        jacobian.setElementAt(2, 14, ftruez);
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

        final double g11 = result[6];
        final double g21 = result[7];
        final double g31 = result[8];
        final double g12 = result[9];
        final double g22 = result[10];
        final double g32 = result[11];
        final double g13 = result[12];
        final double g23 = result[13];
        final double g33 = result[14];

        if (mEstimatedMg == null) {
            mEstimatedMg = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } else {
            mEstimatedMg.initialize(0.0);
        }

        mEstimatedMg.setElementAt(0, 0, sx);

        mEstimatedMg.setElementAt(0, 1, mxy);
        mEstimatedMg.setElementAt(1, 1, sy);

        mEstimatedMg.setElementAt(0, 2, mxz);
        mEstimatedMg.setElementAt(1, 2, myz);
        mEstimatedMg.setElementAt(2, 2, sz);

        if (mEstimatedGg == null) {
            mEstimatedGg = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } else {
            mEstimatedGg.initialize(0.0);
        }

        mEstimatedGg.setElementAtIndex(0, g11);
        mEstimatedGg.setElementAtIndex(1, g21);
        mEstimatedGg.setElementAtIndex(2, g31);
        mEstimatedGg.setElementAtIndex(3, g12);
        mEstimatedGg.setElementAtIndex(4, g22);
        mEstimatedGg.setElementAtIndex(5, g32);
        mEstimatedGg.setElementAtIndex(6, g13);
        mEstimatedGg.setElementAtIndex(7, g23);
        mEstimatedGg.setElementAtIndex(8, g33);

        mEstimatedCovariance = mFitter.getCovar();

        // propagate covariance matrix so that all parameters are taken into
        // account in the order: sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
        // g11, g21, g31, g12, g22, g32, g13, g23, g33

        // We define a lineal function mapping original parameters for the common
        // axis case to the general case
        //[sx'] =   [1  0  0  0  0  0  0  0  0  0  0  0  0  0  0][sx]
        //[sy']     [0  1  0  0  0  0  0  0  0  0  0  0  0  0  0][sy]
        //[sz']     [0  0  1  0  0  0  0  0  0  0  0  0  0  0  0][sz]
        //[mxy']    [0  0  0  1  0  0  0  0  0  0  0  0  0  0  0][mxy]
        //[mxz']    [0  0  0  0  1  0  0  0  0  0  0  0  0  0  0][mxz]
        //[myx']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][myz]
        //[myz']    [0  0  0  0  0  1  0  0  0  0  0  0  0  0  0][g11]
        //[mzx']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][g21]
        //[mzy']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][g31]
        //[g11']    [0  0  0  0  0  0  1  0  0  0  0  0  0  0  0][g12]
        //[g21']    [0  0  0  0  0  0  0  1  0  0  0  0  0  0  0][g22]
        //[g31']    [0  0  0  0  0  0  0  0  1  0  0  0  0  0  0][g32]
        //[g12']    [0  0  0  0  0  0  0  0  0  1  0  0  0  0  0][g13]
        //[g22']    [0  0  0  0  0  0  0  0  0  0  1  0  0  0  0][g23]
        //[g32']    [0  0  0  0  0  0  0  0  0  0  0  1  0  0  0][g33]
        //[g13']    [0  0  0  0  0  0  0  0  0  0  0  0  1  0  0]
        //[g23']    [0  0  0  0  0  0  0  0  0  0  0  0  0  1  0]
        //[g33']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  1]

        // As defined in com.irurueta.statistics.MultivariateNormalDist,
        // if we consider the jacobian of the lineal application the matrix shown
        // above, then covariance can be propagated as follows
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);
        for (int i = 0; i < 5; i++) {
            jacobian.setElementAt(i, i, 1.0);
        }
        jacobian.setElementAt(6, 5, 1.0);

        for (int j = 6, i = 9; j < 15; j++, i++) {
            jacobian.setElementAt(i, j, 1.0);
        }

        // propagated covariance is J * Cov * J'
        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
        mEstimatedChiSq = mFitter.getChisq();
        mEstimatedMse = mFitter.getMse();
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

        // The gyroscope model is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [myx   sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [mzx   mzy    sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        // [Ωmeasx] = [bx] + ( [1+sx  mxy    mxz ]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [myx   1+sy   myz ]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [mzx   mzy    1+sz]  [Ωtruez]   [g31   g32   g33][ftruez]

        // Ωmeasx = bx + (1+sx) * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + (1+sy) * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + (1+sz) * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Where the unknowns are: sx, sy, sz, mxy mxz, myx, myz, mzx, mzy, g11, g12, g13, g21, g22, g23,
        // g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex - bx = sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey - by = myx * Ωtruex + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez - bz = mzx * Ωtruex + mzy * Ωtruey + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [Ωtruex  0       0       Ωtruey  Ωtruez  0       0       0       0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][sx ] =  [Ωmeasx - Ωtruex - bx]
        // [0       Ωtruey  0       0       0       Ωtruex  Ωtruez  0       0       0       0       0       ftruex  ftruey  ftruez  0       0       0     ][sy ]    [Ωmeasy - Ωtruey - by]
        // [0       0       Ωtruez  0       0       0       0       Ωtruex  Ωtruey  0       0       0       0       0       0       ftruex  ftruey  ftruez][sz ]    [Ωmeasz - Ωtruez - bz]
        //                                                                                                                                                 [mxy]
        //                                                                                                                                                 [mxz]
        //                                                                                                                                                 [myx]
        //                                                                                                                                                 [myz]
        //                                                                                                                                                 [mzx]
        //                                                                                                                                                 [mzy]
        //                                                                                                                                                 [g11]
        //                                                                                                                                                 [g12]
        //                                                                                                                                                 [g13]
        //                                                                                                                                                 [g21]
        //                                                                                                                                                 [g22]
        //                                                                                                                                                 [g23]
        //                                                                                                                                                 [g31]
        //                                                                                                                                                 [g32]
        //                                                                                                                                                 [g33]

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true angular rate + specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured angular rate
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

                final double[] buffer = mInitialGg.getBuffer();
                final int num = buffer.length;
                System.arraycopy(buffer, 0, initial, 9, num);

                return initial;
            }

            @Override
            public void evaluate(final int i, final double[] point,
                                 final double[] result, final double[] params,
                                 final Matrix jacobian) {
                // We know that:
                // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
                // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
                // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

                // Hence, the derivatives respect the parameters sx, sy,
                // sz, mxy, mxz, myx, myz, mzx, mzy, g11, g12, g13, g21, g22, g23,
                // g31, g32 and g33 is:

                // d(Ωmeasx)/d(sx) = Ωtruex
                // d(Ωmeasx)/d(sy) = 0.0
                // d(Ωmeasx)/d(sz) = 0.0
                // d(Ωmeasx)/d(mxy) = Ωtruey
                // d(Ωmeasx)/d(mxz) = Ωtruez
                // d(Ωmeasx)/d(myx) = 0.0
                // d(Ωmeasx)/d(myz) = 0.0
                // d(Ωmeasx)/d(mzx) = 0.0
                // d(Ωmeasx)/d(mzy) = 0.0
                // d(Ωmeasx)/d(g11) = ftruex
                // d(Ωmeasx)/d(g12) = ftruey
                // d(Ωmeasx)/d(g13) = ftruez
                // d(Ωmeasx)/d(g21) = 0.0
                // d(Ωmeasx)/d(g22) = 0.0
                // d(Ωmeasx)/d(g23) = 0.0
                // d(Ωmeasx)/d(g31) = 0.0
                // d(Ωmeasx)/d(g32) = 0.0
                // d(Ωmeasx)/d(g33) = 0.0

                // d(Ωmeasy)/d(sx) = 0.0
                // d(Ωmeasy)/d(sy) = Ωtruey
                // d(Ωmeasy)/d(sz) = 0.0
                // d(Ωmeasy)/d(mxy) = 0.0
                // d(Ωmeasy)/d(mxz) = 0.0
                // d(Ωmeasy)/d(myx) = Ωtruex
                // d(Ωmeasy)/d(myz) = Ωtruez
                // d(Ωmeasy)/d(mzx) = 0.0
                // d(Ωmeasy)/d(mzy) = 0.0
                // d(Ωmeasx)/d(g11) = 0.0
                // d(Ωmeasx)/d(g12) = 0.0
                // d(Ωmeasx)/d(g13) = 0.0
                // d(Ωmeasx)/d(g21) = ftruex
                // d(Ωmeasx)/d(g22) = ftruey
                // d(Ωmeasx)/d(g23) = ftruez
                // d(Ωmeasx)/d(g31) = 0.0
                // d(Ωmeasx)/d(g32) = 0.0
                // d(Ωmeasx)/d(g33) = 0.0

                // d(Ωmeasz)/d(sx) = 0.0
                // d(Ωmeasz)/d(sy) = 0.0
                // d(Ωmeasz)/d(sz) = Ωtruez
                // d(Ωmeasz)/d(mxy) = 0.0
                // d(Ωmeasz)/d(mxz) = 0.0
                // d(Ωmeasz)/d(myx) = 0.0
                // d(Ωmeasz)/d(myz) = 0.0
                // d(Ωmeasz)/d(mzx) = Ωtruex
                // d(Ωmeasz)/d(mzy) = Ωtruey
                // d(Ωmeasx)/d(g11) = 0.0
                // d(Ωmeasx)/d(g12) = 0.0
                // d(Ωmeasx)/d(g13) = 0.0
                // d(Ωmeasx)/d(g21) = 0.0
                // d(Ωmeasx)/d(g22) = 0.0
                // d(Ωmeasx)/d(g23) = 0.0
                // d(Ωmeasx)/d(g31) = ftruex
                // d(Ωmeasx)/d(g32) = ftruey
                // d(Ωmeasx)/d(g33) = ftruez

                final double sx = params[0];
                final double sy = params[1];
                final double sz = params[2];

                final double mxy = params[3];
                final double mxz = params[4];
                final double myx = params[5];
                final double myz = params[6];
                final double mzx = params[7];
                final double mzy = params[8];

                final double g11 = params[9];
                final double g21 = params[10];
                final double g31 = params[11];
                final double g12 = params[12];
                final double g22 = params[13];
                final double g32 = params[14];
                final double g13 = params[15];
                final double g23 = params[16];
                final double g33 = params[17];

                final double omegatruex = point[0];
                final double omegatruey = point[1];
                final double omegatruez = point[2];

                final double ftruex = point[3];
                final double ftruey = point[4];
                final double ftruez = point[5];

                result[0] = mBiasX + omegatruex + sx * omegatruex + mxy * omegatruey + mxz * omegatruez
                        + g11 * ftruex + g12 * ftruey + g13 * ftruez;
                result[1] = mBiasY + myx * omegatruex + omegatruey + sy * omegatruey + myz * omegatruez
                        + g21 * ftruex * g22 * ftruey + g23 * ftruez;
                result[2] = mBiasZ + mzx * omegatruex + mzy * omegatruey + omegatruez + sz * omegatruez
                        + g31 * ftruex + g32 * ftruey + g33 * ftruez;

                jacobian.setElementAt(0, 0, omegatruex);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, omegatruey);
                jacobian.setElementAt(0, 4, omegatruez);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, 0.0);
                jacobian.setElementAt(0, 7, 0.0);
                jacobian.setElementAt(0, 8, 0.0);
                jacobian.setElementAt(0, 9, ftruex);
                jacobian.setElementAt(0, 10, ftruey);
                jacobian.setElementAt(0, 11, ftruez);
                jacobian.setElementAt(0, 12, 0.0);
                jacobian.setElementAt(0, 13, 0.0);
                jacobian.setElementAt(0, 14, 0.0);
                jacobian.setElementAt(0, 15, 0.0);
                jacobian.setElementAt(0, 16, 0.0);
                jacobian.setElementAt(0, 17, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, omegatruey);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, 0.0);
                jacobian.setElementAt(1, 5, omegatruex);
                jacobian.setElementAt(1, 6, omegatruez);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, 0.0);
                jacobian.setElementAt(1, 9, 0.0);
                jacobian.setElementAt(1, 10, 0.0);
                jacobian.setElementAt(1, 11, 0.0);
                jacobian.setElementAt(1, 12, ftruex);
                jacobian.setElementAt(1, 13, ftruey);
                jacobian.setElementAt(1, 14, ftruez);
                jacobian.setElementAt(1, 15, 0.0);
                jacobian.setElementAt(1, 16, 0.0);
                jacobian.setElementAt(1, 17, 0.0);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, omegatruez);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, 0.0);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, omegatruex);
                jacobian.setElementAt(2, 8, omegatruey);
                jacobian.setElementAt(2, 9, 0.0);
                jacobian.setElementAt(2, 10, 0.0);
                jacobian.setElementAt(2, 11, 0.0);
                jacobian.setElementAt(2, 12, 0.0);
                jacobian.setElementAt(2, 13, 0.0);
                jacobian.setElementAt(2, 14, 0.0);
                jacobian.setElementAt(2, 15, ftruex);
                jacobian.setElementAt(2, 16, ftruey);
                jacobian.setElementAt(2, 17, ftruez);
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

        final double g11 = result[9];
        final double g21 = result[10];
        final double g31 = result[11];
        final double g12 = result[12];
        final double g22 = result[13];
        final double g32 = result[14];
        final double g13 = result[15];
        final double g23 = result[16];
        final double g33 = result[17];

        if (mEstimatedMg == null) {
            mEstimatedMg = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } else {
            mEstimatedMg.initialize(0.0);
        }

        mEstimatedMg.setElementAt(0, 0, sx);
        mEstimatedMg.setElementAt(1, 0, myx);
        mEstimatedMg.setElementAt(2, 0, mzx);

        mEstimatedMg.setElementAt(0, 1, mxy);
        mEstimatedMg.setElementAt(1, 1, sy);
        mEstimatedMg.setElementAt(2, 1, mzy);

        mEstimatedMg.setElementAt(0, 2, mxz);
        mEstimatedMg.setElementAt(1, 2, myz);
        mEstimatedMg.setElementAt(2, 2, sz);

        if (mEstimatedGg == null) {
            mEstimatedGg = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } else {
            mEstimatedGg.initialize(0.0);
        }

        mEstimatedGg.setElementAtIndex(0, g11);
        mEstimatedGg.setElementAtIndex(1, g21);
        mEstimatedGg.setElementAtIndex(2, g31);
        mEstimatedGg.setElementAtIndex(3, g12);
        mEstimatedGg.setElementAtIndex(4, g22);
        mEstimatedGg.setElementAtIndex(5, g32);
        mEstimatedGg.setElementAtIndex(6, g13);
        mEstimatedGg.setElementAtIndex(7, g23);
        mEstimatedGg.setElementAtIndex(8, g33);

        mEstimatedCovariance = mFitter.getCovar();
        mEstimatedChiSq = mFitter.getChisq();
        mEstimatedMse = mFitter.getMse();
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     */
    private void setInputData() throws WrongSizeException {
        // set input data using:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        //  fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        //  fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
        //  fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

        final BodyKinematics expectedKinematics = new BodyKinematics();

        final int numMeasurements = mMeasurements.size();
        final Matrix x = new Matrix(numMeasurements, 2 * BodyKinematics.COMPONENTS);
        final Matrix y = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final double[] standardDeviations = new double[numMeasurements];
        int i = 0;
        for (final StandardDeviationFrameBodyKinematics measurement : mMeasurements) {
            final BodyKinematics measuredKinematics = measurement.getKinematics();
            final ECEFFrame ecefFrame = measurement.getFrame();
            final ECEFFrame previousEcefFrame = measurement.getPreviousFrame();
            final double timeInterval = measurement.getTimeInterval();

            ECEFKinematicsEstimator.estimateKinematics(timeInterval, ecefFrame,
                    previousEcefFrame, expectedKinematics);

            final double omegaMeasX = measuredKinematics.getAngularRateX();
            final double omegaMeasY = measuredKinematics.getAngularRateY();
            final double omegaMeasZ = measuredKinematics.getAngularRateZ();

            final double omegaTrueX = expectedKinematics.getAngularRateX();
            final double omegaTrueY = expectedKinematics.getAngularRateY();
            final double omegaTrueZ = expectedKinematics.getAngularRateZ();

            final double fTrueX = expectedKinematics.getFx();
            final double fTrueY = expectedKinematics.getFy();
            final double fTrueZ = expectedKinematics.getFz();

            x.setElementAt(i, 0, omegaTrueX);
            x.setElementAt(i, 1, omegaTrueY);
            x.setElementAt(i, 2, omegaTrueZ);

            x.setElementAt(i, 3, fTrueX);
            x.setElementAt(i, 4, fTrueY);
            x.setElementAt(i, 5, fTrueZ);

            y.setElementAt(i, 0, omegaMeasX);
            y.setElementAt(i, 1, omegaMeasY);
            y.setElementAt(i, 2, omegaMeasZ);

            standardDeviations[i] = measurement.getAngularRateStandardDeviation();
            i++;
        }

        mFitter.setInputData(x, y, standardDeviations);
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
}
