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
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSTightlyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.GyroscopeBiasUncertaintySource;
import com.irurueta.navigation.inertial.calibration.GyroscopeCalibrationSource;
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
 * Estimates gyroscope biases, cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * <p>
 * This calibrator uses an iterative approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 7 measurements at different known frames must
 * be provided. In other words, accelerometer and gyroscope (i.e. body kinematics)
 * samples must be obtained at 7 different positions, orientations and velocities
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
public class KnownFrameGyroscopeNonLinearLeastSquaresCalibrator implements
        KnownFrameGyroscopeCalibrator<StandardDeviationFrameBodyKinematics,
                KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener>,
        GyroscopeNonLinearCalibrator, UnknownBiasNonLinearGyroscopeCalibrator,
        GyroscopeCalibrationSource, GyroscopeBiasUncertaintySource {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 7;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 18;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 21;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter mFitter =
            new LevenbergMarquardtMultiVariateFitter();

    /**
     * Initial x-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     */
    private double mInitialBiasX;

    /**
     * Initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     */
    private double mInitialBiasY;

    /**
     * Initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
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
    private KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener mListener;

    /**
     * Estimated angular rate biases for each IMU axis expressed in radians per
     * second (rad/s).
     */
    private double[] mEstimatedBiases;

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
     * Constructor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator() {
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this();
        try {
            setInitialBias(initialBiasX, initialBiasY, initialBiasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per second
     *                     (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per second
     *                       (rad/s).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ) {
        this();
        try {
            setInitialBias(initialBiasX, initialBiasY, initialBiasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ);
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
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listeners to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ);
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
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution. This is expressed in radians per
     *                     second (rad/s).
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution. This is expressed in radians per
     *                       second (rad/s).
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ);
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
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }


    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of gyroscope bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of gyroscope bias to be used
     *                     to find a solution.
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY,
            final AngularSpeed initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param initialBiasX   initial x-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of gyroscope bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of gyroscope bias to be used
     *                       to find a solution.
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
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed initialBiasX,
            final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must have length 3 and is expressed in radians per
     *                    second (rad/s).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double[] initialBias) {
        this();
        try {
            setInitialBias(initialBias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must have length 3 and is expressed in radians per
     *                    second (rad/s).
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double[] initialBias,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial gyroscope bias to be used to find a solution.
     *                     This must have length 3 and is expressed in radians per
     *                     second (rad/s).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double[] initialBias) {
        this(initialBias);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial gyroscope bias to be used to find a solution.
     *                     This must have length 3 and is expressed in radians per
     *                     second (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double[] initialBias,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial gyroscope bias to be used to find a solution.
     *                       This must have length 3 and is expressed in radians per
     *                       second (rad/s).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] initialBias) {
        this(initialBias);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial gyroscope bias to be used to find a solution.
     *                       This must have length 3 and is expressed in radians per
     *                       second (rad/s).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBias);
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
     * @param initialBias    initial gyroscope bias to be used to find a solution.
     *                       This must have length 3 and is expressed in radians per
     *                       second (rad/s).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        this(commonAxisUsed, initialBias);
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
     * @param initialBias    initial gyroscope bias to be used to find a solution.
     *                       This must have length 3 and is expressed in radians per
     *                       second (rad/s).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix initialBias) {
        this();
        try {
            setInitialBias(initialBias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix initialBias,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix initialBias) {
        this(initialBias);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix initialBias,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(initialBias);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBias);
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
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(commonAxisUsed, initialBias);
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
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMg   initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix initialBias, final Matrix initialMg) {
        this(initialBias);
        try {
            setInitialMg(initialMg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMg   initial scale factors and cross coupling errors matrix.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix initialBias, final Matrix initialMg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBias, initialMg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMg) {
        this(initialBias, initialMg);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBias, initialMg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMg) {
        this(initialBias, initialMg);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBias, initialMg);
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
     * @param initialBias    initial bias to find a solution.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMg) {
        this(commonAxisUsed, initialBias, initialMg);
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
     * @param initialBias    initial bias to find a solution.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias, initialMg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMg   initial scale factors and cross coupling errors matrix.
     * @param initialGg   initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg) {
        this(initialBias, initialMg);
        try {
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMg   initial scale factors and cross coupling errors matrix.
     * @param initialGg   initial G-dependent cross biases.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param initialGg    initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg) {
        this(initialBias, initialMg, initialGg);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param initialGg    initial G-dependent cross biases.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg) {
        this(initialBias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBias, initialMg, initialGg);
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
     * @param initialBias    initial bias to find a solution.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg) {
        this(commonAxisUsed, initialBias, initialMg, initialGg);
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
     * @param initialBias    initial bias to find a solution.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    @Override
    public double getInitialBiasX() {
        return mInitialBiasX;
    }

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasX(final double initialBiasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = initialBiasX;
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    @Override
    public double getInitialBiasY() {
        return mInitialBiasY;
    }

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasY(final double initialBiasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasY = initialBiasY;
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    @Override
    public double getInitialBiasZ() {
        return mInitialBiasZ;
    }

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasZ(final double initialBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasZ = initialBiasZ;
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getInitialBiasAngularSpeedX() {
        return new AngularSpeed(mInitialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(mInitialBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasX(final AngularSpeed initialBiasX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = convertAngularSpeed(initialBiasX);
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getInitialBiasAngularSpeedY() {
        return new AngularSpeed(mInitialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(mInitialBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasY(final AngularSpeed initialBiasY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasY = convertAngularSpeed(initialBiasY);
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getInitialBiasAngularSpeedZ() {
        return new AngularSpeed(mInitialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mInitialBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasZ(final AngularSpeed initialBiasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasZ = convertAngularSpeed(initialBiasZ);
    }

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution
     * expressed in radians per second (rad/s).
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
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
     * Sets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBias(final AngularSpeed initialBiasX,
                               final AngularSpeed initialBiasY,
                               final AngularSpeed initialBiasZ)
            throws LockedException {

        if (mRunning) {
            throw new LockedException();
        }
        mInitialBiasX = convertAngularSpeed(initialBiasX);
        mInitialBiasY = convertAngularSpeed(initialBiasY);
        mInitialBiasZ = convertAngularSpeed(initialBiasZ);
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
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of initial bias.
     */
    @Override
    public double[] getInitialBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getInitialBias(result);
        return result;
    }

    /**
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
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
     * Array values are expressed in radians per second (rad/s).
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
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
     * Values are expressed in radians per second (rad/s).
     *
     * @return initial bias to be used to find a solution as a column matrix.
     */
    @Override
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
     * Values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
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
     * Sets initial bias to be used to find a solution as a column matrix
     * with values expressed in radians per second (rad/s).
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
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
     * Gets initial bias coordinates of gyroscope used to find a solution.
     *
     * @return initial bias coordinates.
     */
    @Override
    public AngularSpeedTriad getInitialBiasAsTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                mInitialBiasX, mInitialBiasY, mInitialBiasZ);
    }

    /**
     * Gets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialBiasAsTriad(AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(
                mInitialBiasX, mInitialBiasY, mInitialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param initialBias initial bias coordinates to be set.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBias(AngularSpeedTriad initialBias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInitialBiasX = convertAngularSpeed(
                initialBias.getValueX(), initialBias.getUnit());
        mInitialBiasY = convertAngularSpeed(
                initialBias.getValueY(), initialBias.getUnit());
        mInitialBiasZ = convertAngularSpeed(
                initialBias.getValueZ(), initialBias.getUnit());
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
     * However, if we just want to calibrate a given IMU model (e.g. obtain
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
    @Override
    public KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener getListener() {
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
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener)
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
     * Estimates gyroscope calibration parameters containing bias, scale factors,
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
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return array containing x,y,z components of estimated gyroscope biases.
     */
    @Override
    public double[] getEstimatedBiases() {
        return mEstimatedBiases;
    }

    /**
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @param result instance where estimated gyroscope biases will be stored.
     * @return true if result instance was updated, false otherwise (when estimation
     * is not yet available).
     */
    @Override
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
     * Gets column matrix containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return column matrix containing x,y,z components of estimated gyroscope
     * biases.
     */
    @Override
    public Matrix getEstimatedBiasesAsMatrix() {
        return mEstimatedBiases != null ? Matrix.newFromArray(mEstimatedBiases) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    @Override
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
     * Gets x coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasX() {
        return mEstimatedBiases != null ? mEstimatedBiases[0] : null;
    }

    /**
     * Gets y coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasY() {
        return mEstimatedBiases != null ? mEstimatedBiases[1] : null;
    }

    /**
     * Gets z coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return z coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasZ() {
        return mEstimatedBiases != null ? mEstimatedBiases[2] : null;
    }

    /**
     * Gets x coordinate of estimated gyroscope bias.
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public AngularSpeed getEstimatedBiasAngularSpeedX() {
        return mEstimatedBiases != null ?
                new AngularSpeed(mEstimatedBiases[0],
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets x coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasAngularSpeedX(final AngularSpeed result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[0]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets y coordinate of estimated gyroscope bias.
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public AngularSpeed getEstimatedBiasAngularSpeedY() {
        return mEstimatedBiases != null ?
                new AngularSpeed(mEstimatedBiases[1],
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets y coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasAngularSpeedY(final AngularSpeed result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[1]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets z coordinate of estimated gyroscope bias.
     *
     * @return z coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public AngularSpeed getEstimatedBiasAngularSpeedZ() {
        return mEstimatedBiases != null ?
                new AngularSpeed(mEstimatedBiases[2],
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets z coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasAngularSpeedZ(final AngularSpeed result) {
        if (mEstimatedBiases != null) {
            result.setValue(mEstimatedBiases[2]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated gyroscope bias.
     *
     * @return estimated gyroscope bias or null if not available.
     */
    @Override
    public AngularSpeedTriad getEstimatedBiasAsTriad() {
        return mEstimatedBiases != null ?
                new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                        mEstimatedBiases[0], mEstimatedBiases[1], mEstimatedBiases[2]) : null;
    }

    /**
     * Gets estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated gyroscope bias is available and result was
     * modified, false otherwise.
     */
    @Override
    public boolean getEstimatedBiasAsTriad(final AngularSpeedTriad result) {
        if (mEstimatedBiases != null) {
            result.setValueCoordinatesAndUnit(
                    mEstimatedBiases[0], mEstimatedBiases[1], mEstimatedBiases[2],
                    AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
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
     * Gets estimated covariance matrix for estimated parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): bgx, bgy, bgz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32,
     * gg13, gg23, gg33.
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
     * Gets variance of estimated x coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated x coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasXVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of gyroscope bias expressed in
     * radians per second (rad/s).
     *
     * @return standard deviation of estimated x coordinate of gyroscope bias or null if not
     * available.
     */
    public Double getEstimatedBiasXStandardDeviation() {
        final Double variance = getEstimatedBiasXVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of gyroscope bias.
     *
     * @return standard deviation of estimated x coordinate of gyroscope bias or null if not
     * available.
     */
    public AngularSpeed getEstimatedBiasXStandardDeviationAsAngularSpeed() {
        return mEstimatedCovariance != null ?
                new AngularSpeed(getEstimatedBiasXStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND) :
                null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated x coordinate of gyroscope bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasXStandardDeviationAsAngularSpeed(final AngularSpeed result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasXStandardDeviation());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated y coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated y coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasYVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of gyroscope bias expressed in
     * radians per second (rad/s).
     *
     * @return standard deviation of estimated y coordinate of gyroscope bias or null if not
     * available.
     */
    public Double getEstimatedBiasYStandardDeviation() {
        final Double variance = getEstimatedBiasYVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of gyroscope bias.
     *
     * @return standard deviation of estimated y coordinate of gyroscope bias or null if not
     * available.
     */
    public AngularSpeed getEstimatedBiasYStandardDeviationAsAngularSpeed() {
        return mEstimatedCovariance != null ?
                new AngularSpeed(getEstimatedBiasYStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND) :
                null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated y coordinate of gyroscope bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasYStandardDeviationAsAngularSpeed(final AngularSpeed result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasYStandardDeviation());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated z coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated z coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasZVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of gyroscope bias expressed in
     * radians per second (rad/s).
     *
     * @return standard deviation of estimated z coordinate of gyroscope bias or null if not
     * available.
     */
    public Double getEstimatedBiasZStandardDeviation() {
        final Double variance = getEstimatedBiasZVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of gyroscope bias.
     *
     * @return standard deviation of estimated z coordinate of gyroscope bias or null if not
     * available.
     */
    public AngularSpeed getEstimatedBiasZStandardDeviationAsAngularSpeed() {
        return mEstimatedCovariance != null ?
                new AngularSpeed(getEstimatedBiasZStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND) :
                null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated z coordinate of gyroscope bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasZStandardDeviationAsAngularSpeed(final AngularSpeed result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasZStandardDeviation());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets standard deviation of estimated gyroscope bias coordinates.
     *
     * @return standard deviation of estimated gyroscope bias coordinates.
     */
    public AngularSpeedTriad getEstimatedBiasStandardDeviation() {
        return mEstimatedCovariance != null ?
                new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                        getEstimatedBiasXStandardDeviation(),
                        getEstimatedBiasYStandardDeviation(),
                        getEstimatedBiasZStandardDeviation()) : null;
    }

    /**
     * Gets standard deviation of estimated gyroscope bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of gyroscope bias was available, false
     * otherwise.
     */
    public boolean getEstimatedBiasStandardDeviation(final AngularSpeedTriad result) {
        if (mEstimatedCovariance != null) {
            result.setValueCoordinatesAndUnit(
                    getEstimatedBiasXStandardDeviation(),
                    getEstimatedBiasYStandardDeviation(),
                    getEstimatedBiasZStandardDeviation(),
                    AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets average of estimated standard deviation of gyroscope bias coordinates expressed
     * in radians per second (rad/s).
     *
     * @return average of estimated standard deviation of gyroscope bias coordinates or null
     * if not available.
     */
    public Double getEstimatedBiasStandardDeviationAverage() {
        return mEstimatedCovariance != null ?
                (getEstimatedBiasXStandardDeviation() +
                        getEstimatedBiasYStandardDeviation() +
                        getEstimatedBiasZStandardDeviation()) / 3.0 : null;
    }

    /**
     * Gets average of estimated standard deviation of gyroscope bias coordinates.
     *
     * @return average of estimated standard deviation of gyroscope bias coordinates or null.
     */
    public AngularSpeed getEstimatedBiasStandardDeviationAverageAsAngularSpeed() {
        return mEstimatedCovariance != null ?
                new AngularSpeed(getEstimatedBiasStandardDeviationAverage(),
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets average of estimated standard deviation of gyroscope bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if average of estimated standard deviation of gyroscope bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasStandardDeviationAverageAsAngularSpeed(final AngularSpeed result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasStandardDeviationAverage());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope bias expressed in
     * radians per second (rad/s).
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of gyroscope bias or null
     * if not available.
     */
    @Override
    public Double getEstimatedBiasStandardDeviationNorm() {
        return mEstimatedCovariance != null ?
                Math.sqrt(getEstimatedBiasXVariance() + getEstimatedBiasYVariance() + getEstimatedBiasZVariance()) :
                null;
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope bias.
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of gyroscope bias or null
     * if not available.
     */
    public AngularSpeed getEstimatedBiasStandardDeviationNormAsAngularSpeed() {
        return mEstimatedCovariance != null ?
                new AngularSpeed(getEstimatedBiasStandardDeviationNorm(),
                        AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope bias coordinates.
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @param result instance where result will be stored.
     * @return true if norm of estimated standard deviation of gyroscope bias is
     * available, false otherwise.
     */
    public boolean getEstimatedBiasStandardDeviationNormAsAngularSpeed(final AngularSpeed result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedBiasStandardDeviationNorm());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
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

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myz, g11, g12, g13, g21, g22, g23, g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex = bx + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey = by + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez = bz + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [1   0   0   Ωtruex  0       0       Ωtruey  Ωtruez  0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][bx ] =  [Ωmeasx - Ωtruex]
        // [0   1   0   0       Ωtruey  0       0       0       Ωtruez  0       0       0       ftruex  ftruey  ftruez  0       0       0     ][by ]    [Ωmeasy - Ωtruey]
        // [0   0   1   0       0       Ωtruez  0       0       0       0       0       0       0       0       0       ftruex  ftruey  ftruez][bz ]    [Ωmeasz - Ωtruez]
        //                                                                                                                                              [sx ]
        //                                                                                                                                              [sy ]
        //                                                                                                                                              [sz ]
        //                                                                                                                                              [mxy]
        //                                                                                                                                              [mxz]
        //                                                                                                                                              [myz]
        //                                                                                                                                              [g11]
        //                                                                                                                                              [g12]
        //                                                                                                                                              [g13]
        //                                                                                                                                              [g21]
        //                                                                                                                                              [g22]
        //                                                                                                                                              [g23]
        //                                                                                                                                              [g31]
        //                                                                                                                                              [g32]
        //                                                                                                                                              [g33]

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

                initial[0] = mInitialBiasX;
                initial[1] = mInitialBiasY;
                initial[2] = mInitialBiasZ;

                initial[3] = mInitialSx;
                initial[4] = mInitialSy;
                initial[5] = mInitialSz;

                initial[6] = mInitialMxy;
                initial[7] = mInitialMxz;
                initial[8] = mInitialMyz;

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
                // Ωmeasy = by + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
                // Ωmeasz = bz + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

                // Hence, the derivatives respect the parameters bx, by, bz,
                // sx, sy, sz, mxy mxz, myz, g11, g12, g13, g21, g22, g23,
                // g31, g32, and g33 is:

                // d(Ωmeasx)/d(bx) = 1.0
                // d(Ωmeasx)/d(by) = 0.0
                // d(Ωmeasx)/d(bz) = 0.0
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

                // d(Ωmeasy)/d(bx) = 0.0
                // d(Ωmeasy)/d(by) = 1.0
                // d(Ωmeasy)/d(bz) = 0.0
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

                // d(Ωmeasz)/d(bx) = 0.0
                // d(Ωmeasz)/d(by) = 0.0
                // d(Ωmeasz)/d(bz) = 1.0
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

                final double bx = params[0];
                final double by = params[1];
                final double bz = params[2];

                final double sx = params[3];
                final double sy = params[4];
                final double sz = params[5];

                final double mxy = params[6];
                final double mxz = params[7];
                final double myz = params[8];

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

                result[0] = bx + omegatruex + sx * omegatruex + mxy * omegatruey + mxz * omegatruez
                        + g11 * ftruex + g12 * ftruey + g13 * ftruez;
                result[1] = by + omegatruey + sy * omegatruey + myz * omegatruez
                        + g21 * ftruex * g22 * ftruey + g23 * ftruez;
                result[2] = bz + omegatruez + sz * omegatruez
                        + g31 * ftruex + g32 * ftruey + g33 * ftruez;

                jacobian.setElementAt(0, 0, 1.0);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, omegatruex);
                jacobian.setElementAt(0, 4, 0.0);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, omegatruey);
                jacobian.setElementAt(0, 7, omegatruez);
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
                jacobian.setElementAt(1, 1, 1.0);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, omegatruey);
                jacobian.setElementAt(1, 5, 0.0);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, omegatruez);
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
                jacobian.setElementAt(2, 2, 1.0);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, omegatruez);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, 0.0);
                jacobian.setElementAt(2, 8, 0.0);
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

        final double bx = result[0];
        final double by = result[1];
        final double bz = result[2];

        final double sx = result[3];
        final double sy = result[4];
        final double sz = result[5];

        final double mxy = result[6];
        final double mxz = result[7];
        final double myz = result[8];

        final double g11 = result[9];
        final double g21 = result[10];
        final double g31 = result[11];
        final double g12 = result[12];
        final double g22 = result[13];
        final double g32 = result[14];
        final double g13 = result[15];
        final double g23 = result[16];
        final double g33 = result[17];

        if (mEstimatedBiases == null) {
            mEstimatedBiases = new double[BodyKinematics.COMPONENTS];
        }

        mEstimatedBiases[0] = bx;
        mEstimatedBiases[1] = by;
        mEstimatedBiases[2] = bz;

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
        // account in the order: bx, by, bz, sx, sy, sz, mxy, mxz, myx,
        // myz, mzx, mzy,g11, g21, g31, g12, g22, g32, g13, g23, g33

        // We define a lineal function mapping original parameters for the common
        // axis case to the general case
        //[bx'] =   [1  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][bx]
        //[by']     [0  1  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][by]
        //[bz']     [0  0  1  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][bz]
        //[sx']     [0  0  0  1  0  0  0  0  0  0  0  0  0  0  0  0  0  0][sx]
        //[sy']     [0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0  0  0][sy]
        //[sz']     [0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0  0][sz]
        //[mxy']    [0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0][mxy]
        //[mxz']    [0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0][mxz]
        //[myx']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][myz]
        //[myz']    [0  0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0][g11]
        //[mzx']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][g21]
        //[mzy']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][g31]
        //[g11']    [0  0  0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0][g12]
        //[g21']    [0  0  0  0  0  0  0  0  0  0  1  0  0  0  0  0  0  0][g22]
        //[g31']    [0  0  0  0  0  0  0  0  0  0  0  1  0  0  0  0  0  0][g32]
        //[g12']    [0  0  0  0  0  0  0  0  0  0  0  0  1  0  0  0  0  0][g13]
        //[g22']    [0  0  0  0  0  0  0  0  0  0  0  0  0  1  0  0  0  0][g23]
        //[g32']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  1  0  0  0][g33]
        //[g13']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  1  0  0]
        //[g23']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  1  0]
        //[g33']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  1]

        // As defined in com.irurueta.statistics.MultivariateNormalDist,
        // if we consider the jacobian of the lineal application the matrix shown
        // above, then covariance can be propagated as follows
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);
        for (int i = 0; i < 8; i++) {
            jacobian.setElementAt(i, i, 1.0);
        }
        jacobian.setElementAt(9, 8, 1.0);

        for (int j = 9, i = 12; j < 18; j++, i++) {
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

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myx, myz, mzx, mzy, g11, g12, g13, g21, g22, g23,
        // g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex = bx + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey = by + myx * Ωtruex + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez = bz + mzx * Ωtruex + mzy * Ωtruey + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [1   0   0   Ωtruex  0       0       Ωtruey  Ωtruez  0       0       0       0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][bx ] =  [Ωmeasx - Ωtruex]
        // [0   1   0   0       Ωtruey  0       0       0       Ωtruex  Ωtruez  0       0       0       0       0       ftruex  ftruey  ftruez  0       0       0     ][by ]    [Ωmeasy - Ωtruey]
        // [0   0   1   0       0       Ωtruez  0       0       0       0       Ωtruex  Ωtruey  0       0       0       0       0       0       ftruex  ftruey  ftruez][bz ]    [Ωmeasz - Ωtruez]
        //                                                                                                                                                             [sx ]
        //                                                                                                                                                             [sy ]
        //                                                                                                                                                             [sz ]
        //                                                                                                                                                             [mxy]
        //                                                                                                                                                             [mxz]
        //                                                                                                                                                             [myx]
        //                                                                                                                                                             [myz]
        //                                                                                                                                                             [mzx]
        //                                                                                                                                                             [mzy]
        //                                                                                                                                                             [g11]
        //                                                                                                                                                             [g12]
        //                                                                                                                                                             [g13]
        //                                                                                                                                                             [g21]
        //                                                                                                                                                             [g22]
        //                                                                                                                                                             [g23]
        //                                                                                                                                                             [g31]
        //                                                                                                                                                             [g32]
        //                                                                                                                                                             [g33]

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
                final double[] initial = new double[GENERAL_UNKNOWNS];

                initial[0] = mInitialBiasX;
                initial[1] = mInitialBiasY;
                initial[2] = mInitialBiasZ;

                initial[3] = mInitialSx;
                initial[4] = mInitialSy;
                initial[5] = mInitialSz;

                initial[6] = mInitialMxy;
                initial[7] = mInitialMxz;
                initial[8] = mInitialMyx;
                initial[9] = mInitialMyz;
                initial[10] = mInitialMzx;
                initial[11] = mInitialMzy;

                final double[] buffer = mInitialGg.getBuffer();
                final int num = buffer.length;
                System.arraycopy(buffer, 0, initial, 12, num);

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

                // Hence, the derivatives respect the parameters bx, by, bz, sx, sy,
                // sz, mxy, mxz, myx, myz, mzx, mzy, g11, g12, g13, g21, g22, g23,
                // g31, g32 and g33 is:

                // d(Ωmeasx)/d(bx) = 1.0
                // d(Ωmeasx)/d(by) = 0.0
                // d(Ωmeasx)/d(bz) = 0.0
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

                // d(Ωmeasy)/d(bx) = 0.0
                // d(Ωmeasy)/d(by) = 1.0
                // d(Ωmeasy)/d(bz) = 0.0
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

                // d(Ωmeasz)/d(bx) = 0.0
                // d(Ωmeasz)/d(by) = 0.0
                // d(Ωmeasz)/d(bz) = 1.0
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

                final double bx = params[0];
                final double by = params[1];
                final double bz = params[2];

                final double sx = params[3];
                final double sy = params[4];
                final double sz = params[5];

                final double mxy = params[6];
                final double mxz = params[7];
                final double myx = params[8];
                final double myz = params[9];
                final double mzx = params[10];
                final double mzy = params[11];

                final double g11 = params[12];
                final double g21 = params[13];
                final double g31 = params[14];
                final double g12 = params[15];
                final double g22 = params[16];
                final double g32 = params[17];
                final double g13 = params[18];
                final double g23 = params[19];
                final double g33 = params[20];

                final double omegatruex = point[0];
                final double omegatruey = point[1];
                final double omegatruez = point[2];

                final double ftruex = point[3];
                final double ftruey = point[4];
                final double ftruez = point[5];

                result[0] = bx + omegatruex + sx * omegatruex + mxy * omegatruey + mxz * omegatruez
                        + g11 * ftruex + g12 * ftruey + g13 * ftruez;
                result[1] = by + myx * omegatruex + omegatruey + sy * omegatruey + myz * omegatruez
                        + g21 * ftruex * g22 * ftruey + g23 * ftruez;
                result[2] = bz + mzx * omegatruex + mzy * omegatruey + omegatruez + sz * omegatruez
                        + g31 * ftruex + g32 * ftruey + g33 * ftruez;

                jacobian.setElementAt(0, 0, 1.0);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, omegatruex);
                jacobian.setElementAt(0, 4, 0.0);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, omegatruey);
                jacobian.setElementAt(0, 7, omegatruez);
                jacobian.setElementAt(0, 8, 0.0);
                jacobian.setElementAt(0, 9, 0.0);
                jacobian.setElementAt(0, 10, 0.0);
                jacobian.setElementAt(0, 11, 0.0);
                jacobian.setElementAt(0, 12, ftruex);
                jacobian.setElementAt(0, 13, ftruey);
                jacobian.setElementAt(0, 14, ftruez);
                jacobian.setElementAt(0, 15, 0.0);
                jacobian.setElementAt(0, 16, 0.0);
                jacobian.setElementAt(0, 17, 0.0);
                jacobian.setElementAt(0, 18, 0.0);
                jacobian.setElementAt(0, 19, 0.0);
                jacobian.setElementAt(0, 20, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, 1.0);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, omegatruey);
                jacobian.setElementAt(1, 5, 0.0);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, omegatruex);
                jacobian.setElementAt(1, 9, omegatruez);
                jacobian.setElementAt(1, 10, 0.0);
                jacobian.setElementAt(1, 11, 0.0);
                jacobian.setElementAt(1, 12, 0.0);
                jacobian.setElementAt(1, 13, 0.0);
                jacobian.setElementAt(1, 14, 0.0);
                jacobian.setElementAt(1, 15, ftruex);
                jacobian.setElementAt(1, 16, ftruey);
                jacobian.setElementAt(1, 17, ftruez);
                jacobian.setElementAt(1, 18, 0.0);
                jacobian.setElementAt(1, 19, 0.0);
                jacobian.setElementAt(1, 20, 0.0);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, 1.0);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, omegatruez);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, 0.0);
                jacobian.setElementAt(2, 8, 0.0);
                jacobian.setElementAt(2, 9, 0.0);
                jacobian.setElementAt(2, 10, omegatruex);
                jacobian.setElementAt(2, 11, omegatruey);
                jacobian.setElementAt(2, 12, 0.0);
                jacobian.setElementAt(2, 13, 0.0);
                jacobian.setElementAt(2, 14, 0.0);
                jacobian.setElementAt(2, 15, 0.0);
                jacobian.setElementAt(2, 16, 0.0);
                jacobian.setElementAt(2, 17, 0.0);
                jacobian.setElementAt(2, 18, ftruex);
                jacobian.setElementAt(2, 19, ftruey);
                jacobian.setElementAt(2, 20, ftruez);
            }
        });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double bx = result[0];
        final double by = result[1];
        final double bz = result[2];

        final double sx = result[3];
        final double sy = result[4];
        final double sz = result[5];

        final double mxy = result[6];
        final double mxz = result[7];
        final double myx = result[8];
        final double myz = result[9];
        final double mzx = result[10];
        final double mzy = result[11];

        final double g11 = result[12];
        final double g21 = result[13];
        final double g31 = result[14];
        final double g12 = result[15];
        final double g22 = result[16];
        final double g32 = result[17];
        final double g13 = result[18];
        final double g23 = result[19];
        final double g33 = result[20];

        if (mEstimatedBiases == null) {
            mEstimatedBiases = new double[BodyKinematics.COMPONENTS];
        }

        mEstimatedBiases[0] = bx;
        mEstimatedBiases[1] = by;
        mEstimatedBiases[2] = bz;

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
     * Converts angular speed value and unit to radians per second.
     *
     * @param value angular speed value.
     * @param unit unit of angular speed value.
     * @return converted value.
     */
    private static double convertAngularSpeed(final double value, final AngularSpeedUnit unit) {
        return AngularSpeedConverter.convert(value, unit, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Converts angular speed instance to radians per second.
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return convertAngularSpeed(angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit());
    }
}
