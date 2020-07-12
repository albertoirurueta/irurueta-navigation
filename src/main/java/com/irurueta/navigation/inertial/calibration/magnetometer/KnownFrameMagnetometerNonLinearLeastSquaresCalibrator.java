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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;

import java.io.IOException;
import java.util.Collection;

/**
 * Estimates magnetometer hard-iron biases, cross couplings and scaling
 * factors.
 * <p>
 * This calibrator uses an iterative approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 4 measurements at different known frames must
 * be provided. In other words, magnetometer samples must be obtained at 4
 * different positions or orientations.
 * Notice that frame velocities are ignored by this calibrator.
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
 */
public class KnownFrameMagnetometerNonLinearLeastSquaresCalibrator implements
        KnownFrameMagnetometerCalibrator<StandardDeviationFrameBodyMagneticFluxDensity,
                KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener>,
        MagnetometerNonLinearCalibrator, UnknownHardIronNonLinearMagnetometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 4;

    /**
     * Number of unknowns when common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 9;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 12;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter mFitter =
            new LevenbergMarquardtMultiVariateFitter();

    /**
     * Initial x-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mInitialHardIronX;

    /**
     * Initial y-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mInitialHardIronY;

    /**
     * Initial z-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mInitialHardIronZ;

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
     * Contains a collection of body magnetic flux density measurements taken
     * at different frames (positions and orientations) and containing the
     * standard deviation of magnetometer measurements.
     * If a single device magnetometer needs to be calibrated, typically all
     * measurements are taken at the same position, with zero velocity and
     * multiple orientations.
     * However, if we just want to calibrate a given magnetometer model (e.g.
     * obtain an average and less precise calibration for the magnetometer of
     * a given phone model), we could take measurements collected throughout
     * the planet at multiple positions while the phone remains static (e.g.
     * while charging), hence each measurement position will change, velocity
     * will remain zero and orientation will be typically constant at
     * horizontal orientation while the phone remains on a
     * flat surface.
     */
    private Collection<? extends StandardDeviationFrameBodyMagneticFluxDensity> mMeasurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from Mm matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener mListener;

    /**
     * Estimated magnetometer hard-iron biases for each magnetometer axis
     * expressed in Teslas (T).
     */
    private double[] mEstimatedHardIron;

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
     * Estimated covariance matrix for estimated parameters.
     */
    private Matrix mEstimatedCovariance;

    /**
     * Estimated chi square value.
     */
    private double mEstimatedChiSq;

    /**
     * Indicates whether calibrator is running.
     */
    private boolean mRunning;

    /**
     * Contains Earth's magnetic model.
     */
    private WorldMagneticModel mMagneticModel;

    /**
     * Constructor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel) {
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel) {
        this(measurements);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel) {
        this(commonAxisUsed);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel) {
        this(measurements, commonAxisUsed);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        try {
            setInitialHardIron(initialHardIronX, initialHardIronY,
                    initialHardIronZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIronX, initialHardIronY, initialHardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY,
                initialHardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY,
                initialHardIronZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIronX, initialHardIronY,
                initialHardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        mMeasurements = measurements;
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX,
            final double initialHardIronY,
            final double initialHardIronZ) {
        this(magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX,
            final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIronX,
                initialHardIronY, initialHardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX,
            final double initialHardIronY,
            final double initialHardIronZ) {
        this(commonAxisUsed, magneticModel, initialHardIronX,
                initialHardIronY, initialHardIronZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX,
            final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIronX,
                initialHardIronY, initialHardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIronX,
                initialHardIronY, initialHardIronZ, initialSx,
                initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(commonAxisUsed, magneticModel, initialHardIronX,
                initialHardIronY, initialHardIronZ, initialSx, initialSy,
                initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIronX,
                initialHardIronY, initialHardIronZ, initialSx, initialSy,
                initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        try {
            setInitialScalingFactorsAndCrossCouplingErrors(initialSx,
                    initialSy, initialSz, initialMxy, initialMxz,
                    initialMyx, initialMyz, initialMzx, initialMzy);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx,
                initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY,
                initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIronX,
                initialHardIronY, initialHardIronZ, initialSx, initialSy,
                initialSz, initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double[] initialHardIron) {
        try {
            setInitialHardIron(initialHardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this
     *                        calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        this(initialHardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double[] initialHardIron) {
        this(initialHardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double[] initialHardIron) {
        this(commonAxisUsed, initialHardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double[] initialHardIron) {
        this(initialHardIron);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double[] initialHardIron) {
        this(magneticModel, initialHardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double[] initialHardIron) {
        this(magneticModel, initialHardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double[] initialHardIron) {
        this(commonAxisUsed, magneticModel, initialHardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Matrix initialHardIron) {
        try {
            setInitialHardIron(initialHardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this
     *                        calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        this(initialHardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final Matrix initialHardIron) {
        this(initialHardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final Matrix initialHardIron) {
        this(commonAxisUsed, initialHardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron) {
        this(initialHardIron);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron) {
        this(magneticModel, initialHardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron) {
        this(magneticModel, initialHardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron) {
        this(commonAxisUsed, magneticModel, initialHardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron);
        try {
            setInitialMm(initialMm);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this
     *                        calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron, initialMm);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron, initialMm);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(commonAxisUsed, initialHardIron, initialMm);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron, initialMm);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(magneticModel, initialHardIron, initialMm);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(magneticModel, initialHardIron, initialMm);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(commonAxisUsed, magneticModel, initialHardIron, initialMm);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIron,
                initialMm);
        mListener = listener;
    }

    /**
     * Gets initial x-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial x-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getInitialHardIronX() {
        return mInitialHardIronX;
    }

    /**
     * Sets initial x-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronX(final double initialHardIronX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronX = initialHardIronX;
    }

    /**
     * Gets initial y-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial y-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getInitialHardIronY() {
        return mInitialHardIronY;
    }

    /**
     * Sets initial y-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronY(final double initialHardIronY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronY = initialHardIronY;
    }

    /**
     * Gets initial z-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial z-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getInitialHardIronZ() {
        return mInitialHardIronZ;
    }

    /**
     * Sets initial z-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in meters Teslas (T).
     *
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronZ(final double initialHardIronZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronZ = initialHardIronZ;
    }

    /**
     * Sets initial hard-iron bias coordinates of magnetometer used to find
     * a solution expressed in Teslas (T).
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias.
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias.
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIron(
            final double initialHardIronX,
            final double initialHardIronY,
            final double initialHardIronZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronX = initialHardIronX;
        mInitialHardIronY = initialHardIronY;
        mInitialHardIronZ = initialHardIronZ;
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
    @Override
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
    @Override
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
    @Override
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
     * Gets initial hard-iron bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @return array containing coordinates of initial bias.
     */
    @Override
    public double[] getInitialHardIron() {
        final double[] result = new double[
                BodyMagneticFluxDensity.COMPONENTS];
        getInitialHardIron(result);
        return result;
    }

    /**
     * Gets initial hard-iron bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    @Override
    public void getInitialHardIron(final double[] result) {
        if (result.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = mInitialHardIronX;
        result[1] = mInitialHardIronY;
        result[2] = mInitialHardIronZ;
    }

    /**
     * Sets initial hard-iron bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void setInitialHardIron(final double[] initialHardIron)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (initialHardIron.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        mInitialHardIronX = initialHardIron[0];
        mInitialHardIronY = initialHardIron[1];
        mInitialHardIronZ = initialHardIron[2];
    }

    /**
     * Gets initial hard-iron bias to be used to find a solution as a
     * column matrix.
     *
     * @return initial hard-iron bias to be used to find a solution as a
     * column matrix.
     */
    @Override
    public Matrix getInitialHardIronAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    1);
            getInitialHardIronAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets initial hard-iron bias to be used to find a solution as a
     * column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void getInitialHardIronAsMatrix(final Matrix result) {
        if (result.getRows() != BodyMagneticFluxDensity.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mInitialHardIronX);
        result.setElementAtIndex(1, mInitialHardIronY);
        result.setElementAtIndex(2, mInitialHardIronZ);
    }

    /**
     * Sets initial hard-iron bias to be used to find a solution.
     *
     * @param initialHardIron initial hard-iron bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setInitialHardIron(final Matrix initialHardIron)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialHardIron.getRows() != BodyMagneticFluxDensity.COMPONENTS
                || initialHardIron.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mInitialHardIronX = initialHardIron.getElementAtIndex(0);
        mInitialHardIronY = initialHardIron.getElementAtIndex(1);
        mInitialHardIronZ = initialHardIron.getElementAtIndex(2);
    }

    /**
     * Gets initial scale factors and cross coupling errors matrix.
     *
     * @return initial scale factors and cross coupling errors matrix.
     */
    @Override
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
    @Override
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
    @Override
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
     * Gets a collection of body magnetic flux density measurements taken at different
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
     * @return a collection of body magnetic flux density measurements taken at different
     * frames (positions, orientations and velocities).
     */
    @Override
    public Collection<? extends StandardDeviationFrameBodyMagneticFluxDensity> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets a collection of body magnetic flux density measurements taken at different
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
     * @param measurements collection of body magnetic flux density measurements
     *                     taken at different frames (positions, orientations
     *                     and velocities).
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setMeasurements(
            final Collection<? extends StandardDeviationFrameBodyMagneticFluxDensity> measurements)
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
    @Override
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
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener getListener() {
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
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Indicates whether calibrator is ready to start the estimator.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mMeasurements != null && mMeasurements.size() >= MINIMUM_MEASUREMENTS;
    }

    /**
     * Indicates whether calibrator is currently running or no.
     *
     * @return true if calibrator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Gets Earth's magnetic model.
     *
     * @return Earth's magnetic model or null if not provided.
     */
    @Override
    public WorldMagneticModel getMagneticModel() {
        return mMagneticModel;
    }

    /**
     * Sets Earth's magnetic model.
     *
     * @param magneticModel Earth's magnetic model to be set.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setMagneticModel(final WorldMagneticModel magneticModel)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMagneticModel = magneticModel;
    }

    /**
     * Estimates magnetometer calibration parameters containing scale factors
     * and cross-coupling errors.
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

        } catch (final AlgebraException | FittingException |
                com.irurueta.numerical.NotReadyException |
                IOException e) {
            throw new CalibrationException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Gets array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @return array containing x,y,z components of estimated magnetometer
     * hard-iron biases.
     */
    @Override
    public double[] getEstimatedHardIron() {
        return mEstimatedHardIron;
    }

    /**
     * Gets array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @param result instance where estimated magnetometer biases will be
     *               stored.
     * @return true if result instance was updated, false otherwise (when
     * estimation is not yet available).
     */
    @Override
    public boolean getEstimatedHardIron(final double[] result) {
        if (mEstimatedHardIron != null) {
            System.arraycopy(mEstimatedHardIron, 0, result,
                    0, mEstimatedHardIron.length);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases expressed in Teslas (T).
     *
     * @return column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases.
     */
    @Override
    public Matrix getEstimatedHardIronAsMatrix() {
        return mEstimatedHardIron != null ? Matrix.newFromArray(mEstimatedHardIron) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases expressed in Teslas (T).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    @Override
    public boolean getEstimatedHardIronAsMatrix(final Matrix result)
            throws WrongSizeException {
        if (mEstimatedHardIron != null) {
            result.fromArray(mEstimatedHardIron);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets x coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return x coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronX() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[0] : null;
    }

    /**
     * Gets y coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return y coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronY() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[1] : null;
    }

    /**
     * Gets z coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return z coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronZ() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[2] : null;
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
    @Override
    public Matrix getEstimatedMm() {
        return mEstimatedMm;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return mEstimatedMm != null ?
                mEstimatedMm.getElementAt(2, 1) : null;
    }

    /**
     * Gets estimated covariance matrix for estimated position.
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
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException                         if there are numerical errors.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     * @throws IOException                              if world magnetic model cannot be loaded.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException, IOException {
        // The accelerometer model is:
        // mBmeas = ba + (I + Ma) * mBtrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // mBmeas = ba + (I + Ma) * mBtrue

        // Hence:
        //  [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        //  [mBmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [mBtruey]
        //  [mBmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [mBtruez]

        // where myx = mzx = mzy = 0

        // Hence:
        //  [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        //  [mBmeasy] = [by]     [0  1   0]   [0     sy  myz]    [mBtruey]
        //  [mBmeasz] = [bz]     [0  0   1]   [0     0   sz ]    [mBtruez]

        //  [mBmeasx] = [bx] +   [1+sx   mxy     mxz ][mBtruex]
        //  [mBmeasy]   [by]     [0      1+sy    myz ][mBtruey]
        //  [mBmeasz]   [bz]     [0      0       1+sz][mBtruez]

        //  mBmeasx = bx + (1+sx) * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + (1+sy) * mBtruey + myz * mBtruez
        //  mBmeasz = bz + (1+sz) * mBtruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myz
        // Reordering:
        //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + mBtruey + sy * mBtruey + myz * mBtruez
        //  mBmeasz = bz + mBtruez + sz * mBtruez

        //  mBmeasx - mBtruex = bx + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy - mBtruey = by + sy * mBtruey + myz * mBtruez
        //  mBmeasz - mBtruez = bz + sz * mBtruez

        // [1   0   0   mBtruex  0        0        mBtruey  mBtruez  0      ][bx ] = [mBmeasx - mBtruex]
        // [0   1   0   0        mBtruey  0        0        0        mBtruez][by ]   [mBmeasy - mBtruey]
        // [0   0   1   0        0        mBtruez  0        0        0      ][bz ]   [mBmeasz - mBtruez]
        //                                                                   [sx ]
        //                                                                   [sy ]
        //                                                                   [sz ]
        //                                                                   [mxy]
        //                                                                   [mxz]
        //                                                                   [myz]

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true magnetic flux density coordinates
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured magnetic flux density
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                initial[0] = mInitialHardIronX;
                initial[1] = mInitialHardIronY;
                initial[2] = mInitialHardIronZ;

                initial[3] = mInitialSx;
                initial[4] = mInitialSy;
                initial[5] = mInitialSz;

                initial[6] = mInitialMxy;
                initial[7] = mInitialMxz;
                initial[8] = mInitialMyz;

                return initial;
            }

            @Override
            public void evaluate(int i, double[] point, double[] result, double[] params, Matrix jacobian) {
                // We know that:
                //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
                //  mBmeasy = by + mBtruey + sy * mBtruey + myz * mBtruez
                //  mBmeasz = bz + mBtruez + sz * mBtruez

                // Hence, the derivatives respect the parameters bx, by, bz, sx, sy,
                // sz, mxy, mxz, myz

                // d(fmeasx)/d(bx) = 1.0
                // d(fmeasx)/d(by) = 0.0
                // d(fmeasx)/d(bz) = 0.0
                // d(fmeasx)/d(sx) = mBtruex
                // d(fmeasx)/d(sy) = 0.0
                // d(fmeasx)/d(sz) = 0.0
                // d(fmeasx)/d(mxy) = mBtruey
                // d(fmeasx)/d(mxz) = mBtruez
                // d(fmeasx)/d(myz) = 0.0

                // d(fmeasy)/d(bx) = 0.0
                // d(fmeasy)/d(by) = 1.0
                // d(fmeasy)/d(bz) = 0.0
                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = mBtruey
                // d(fmeasy)/d(sz) = 0.0
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myz) = mBtruez

                // d(fmeasz)/d(bx) = 0.0
                // d(fmeasz)/d(by) = 0.0
                // d(fmeasz)/d(bz) = 1.0
                // d(fmeasz)/d(sx) = 0.0
                // d(fmeasz)/d(sy) = 0.0
                // d(fmeasz)/d(sz) = mBtruez
                // d(fmeasz)/d(mxy) = 0.0
                // d(fmeasz)/d(mxz) = 0.0
                // d(fmeasz)/d(myz) = 0.0

                final double bx = params[0];
                final double by = params[1];
                final double bz = params[2];

                final double sx = params[3];
                final double sy = params[4];
                final double sz = params[5];

                final double mxy = params[6];
                final double mxz = params[7];
                final double myz = params[8];

                final double btruex = point[0];
                final double btruey = point[1];
                final double btruez = point[2];

                result[0] = bx + btruex + sx * btruex + mxy * btruey + mxz * btruez;
                result[1] = by + btruey + sy * btruey + myz * btruez;
                result[2] = bz + btruez + sz * btruez;

                jacobian.setElementAt(0, 0, 1.0);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, btruex);
                jacobian.setElementAt(0, 4, 0.0);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, btruey);
                jacobian.setElementAt(0, 7, btruez);
                jacobian.setElementAt(0, 8, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, 1.0);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, btruey);
                jacobian.setElementAt(1, 5, 0.0);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, btruez);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, 1.0);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, btruez);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, 0.0);
                jacobian.setElementAt(2, 8, 0.0);
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

        if (mEstimatedHardIron == null) {
            mEstimatedHardIron = new double[
                    BodyMagneticFluxDensity.COMPONENTS];
        }

        mEstimatedHardIron[0] = bx;
        mEstimatedHardIron[1] = by;
        mEstimatedHardIron[2] = bz;

        if (mEstimatedMm == null) {
            mEstimatedMm = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
        } else {
            mEstimatedMm.initialize(0.0);
        }

        mEstimatedMm.setElementAt(0, 0, sx);

        mEstimatedMm.setElementAt(0, 1, mxy);
        mEstimatedMm.setElementAt(1, 1, sy);

        mEstimatedMm.setElementAt(0, 2, mxz);
        mEstimatedMm.setElementAt(1, 2, myz);
        mEstimatedMm.setElementAt(2, 2, sz);

        mEstimatedCovariance = mFitter.getCovar();
        mEstimatedChiSq = mFitter.getChisq();
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws AlgebraException                         if there are numerical errors.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     * @throws IOException                              if world magnetic model cannot be loaded.
     */
    private void calibrateGeneral() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException, IOException {
        // The magnetometer model is:
        // mBmeas = ba + (I + Mm) * mBtrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // mBmeas = ba + (I + Mm) * mBtrue

        // Hence:
        //  [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        //  [mBmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [mBtruey]
        //  [mBmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [mBtruez]

        //  [mBmeasx] = [bx] +   [1+sx   mxy     mxz ][mBtruex]
        //  [mBmeasy]   [by]     [myx    1+sy    myz ][mBtruey]
        //  [mBmeasz]   [bz]     [mzx    mzy     1+sz][mBtruez]

        //  mBmeasx = bx + (1+sx) * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + myx * mBtruex + (1+sy) * mBtruey + myz * mBtruez
        //  mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + (1+sz) * mBtruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myx, myz, mzx, mzy
        // Reordering:
        //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + myx * mBtruex + mBtruey + sy * mBtruey + myz * mBtruez
        //  mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + mBtruez + sz * mBtruez

        //  mBmeasx - mBtruex = bx + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy - mBtruey = by + myx * mBtruex + sy * mBtruey + myz * mBtruez
        //  mBmeasz - mBtruez = bz + mzx * mBtruex + mzy * mBtruey + sz * mBtruez

        // [1   0   0   mBtruex  0        0        mBtruey  mBtruez  0        0        0        0      ][bx ] = [mBmeasx - mBtruex]
        // [0   1   0   0        mBtruey  0        0        0        mBtruex  mBtruez  0        0      ][by ]   [mBmeasy - mBtruey]
        // [0   0   1   0        0        mBtruez  0        0        0        0        mBtruex  mBtruey][bz ]   [mBmeasz - mBtruez]
        //                                                                                              [sx ]
        //                                                                                              [sy ]
        //                                                                                              [sz ]
        //                                                                                              [mxy]
        //                                                                                              [mxz]
        //                                                                                              [myx]
        //                                                                                              [myz]
        //                                                                                              [mzx]
        //                                                                                              [mzy]

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true magnetic flux density coordinates
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured magnetic flux density
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[GENERAL_UNKNOWNS];

                initial[0] = mInitialHardIronX;
                initial[1] = mInitialHardIronY;
                initial[2] = mInitialHardIronZ;

                initial[3] = mInitialSx;
                initial[4] = mInitialSy;
                initial[5] = mInitialSz;

                initial[6] = mInitialMxy;
                initial[7] = mInitialMxz;
                initial[8] = mInitialMyx;
                initial[9] = mInitialMyz;
                initial[10] = mInitialMzx;
                initial[11] = mInitialMzy;

                return initial;
            }

            @Override
            public void evaluate(final int i, final double[] point,
                                 final double[] result, final double[] params,
                                 final Matrix jacobian) {
                // We know that:
                //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
                //  mBmeasy = by + myx * mBtruex + mBtruey + sy * mBtruey + myz * mBtruez
                //  mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + mBtruez + sz * mBtruez

                // Hence, the derivatives respect the parameters bx, by, bz, sx, sy,
                // sz, mxy, mxz, myx, myz, mzx and mzy is:

                // d(fmeasx)/d(bx) = 1.0
                // d(fmeasx)/d(by) = 0.0
                // d(fmeasx)/d(bz) = 0.0
                // d(fmeasx)/d(sx) = mBtruex
                // d(fmeasx)/d(sy) = 0.0
                // d(fmeasx)/d(sz) = 0.0
                // d(fmeasx)/d(mxy) = mBtruey
                // d(fmeasx)/d(mxz) = mBtruez
                // d(fmeasx)/d(myx) = 0.0
                // d(fmeasx)/d(myz) = 0.0
                // d(fmeasx)/d(mzx) = 0.0
                // d(fmeasx)/d(mzy) = 0.0

                // d(fmeasy)/d(bx) = 0.0
                // d(fmeasy)/d(by) = 1.0
                // d(fmeasy)/d(bz) = 0.0
                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = mBtruey
                // d(fmeasy)/d(sz) = 0.0
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myx) = mBtruex
                // d(fmeasy)/d(myz) = mBtruez
                // d(fmeasy)/d(mzx) = 0.0
                // d(fmeasy)/d(mzy) = 0.0

                // d(fmeasz)/d(bx) = 0.0
                // d(fmeasz)/d(by) = 0.0
                // d(fmeasz)/d(bz) = 1.0
                // d(fmeasz)/d(sx) = 0.0
                // d(fmeasz)/d(sy) = 0.0
                // d(fmeasz)/d(sz) = mBtruez
                // d(fmeasz)/d(mxy) = 0.0
                // d(fmeasz)/d(mxz) = 0.0
                // d(fmeasz)/d(myx) = 0.0
                // d(fmeasz)/d(myz) = 0.0
                // d(fmeasz)/d(mzx) = mBtruex
                // d(fmeasz)/d(mzy) = mBtruey

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

                final double btruex = point[0];
                final double btruey = point[1];
                final double btruez = point[2];

                result[0] = bx + btruex + sx * btruex + mxy * btruey + mxz * btruez;
                result[1] = by + myx * btruex + btruey + sy * btruey + myz * btruez;
                result[2] = bz + mzx * btruex + mzy * btruey + btruez + sz * btruez;

                jacobian.setElementAt(0, 0, 1.0);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, btruex);
                jacobian.setElementAt(0, 4, 0.0);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, btruey);
                jacobian.setElementAt(0, 7, btruez);
                jacobian.setElementAt(0, 8, 0.0);
                jacobian.setElementAt(0, 9, 0.0);
                jacobian.setElementAt(0, 10, 0.0);
                jacobian.setElementAt(0, 11, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, 1.0);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, btruey);
                jacobian.setElementAt(1, 5, 0.0);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, btruex);
                jacobian.setElementAt(1, 9, btruez);
                jacobian.setElementAt(1, 10, 0.0);
                jacobian.setElementAt(1, 11, 0.0);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, 1.0);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, btruez);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, 0.0);
                jacobian.setElementAt(2, 8, 0.0);
                jacobian.setElementAt(2, 9, 0.0);
                jacobian.setElementAt(2, 10, btruex);
                jacobian.setElementAt(2, 11, btruey);
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

        if (mEstimatedHardIron == null) {
            mEstimatedHardIron = new double[
                    BodyMagneticFluxDensity.COMPONENTS];
        }

        mEstimatedHardIron[0] = bx;
        mEstimatedHardIron[1] = by;
        mEstimatedHardIron[2] = bz;

        if (mEstimatedMm == null) {
            mEstimatedMm = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS);
        } else {
            mEstimatedMm.initialize(0.0);
        }

        mEstimatedMm.setElementAt(0, 0, sx);
        mEstimatedMm.setElementAt(1, 0, myx);
        mEstimatedMm.setElementAt(2, 0, mzx);

        mEstimatedMm.setElementAt(0, 1, mxy);
        mEstimatedMm.setElementAt(1, 1, sy);
        mEstimatedMm.setElementAt(2, 1, mzy);

        mEstimatedMm.setElementAt(0, 2, mxz);
        mEstimatedMm.setElementAt(1, 2, myz);
        mEstimatedMm.setElementAt(2, 2, sz);

        mEstimatedCovariance = mFitter.getCovar();
        mEstimatedChiSq = mFitter.getChisq();
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     * @throws IOException        if world magnetic model cannot be loaded.
     */
    private void setInputData() throws WrongSizeException, IOException {
        // set input data using:
        //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + myx * mBtruex + mBtruey + sy * mBtruey + myz * mBtruez
        //  mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + mBtruez + sz * mBtruez

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator;
        if (mMagneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(mMagneticModel);
        } else {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }

        final BodyMagneticFluxDensity expectedMagneticFluxDensity = new BodyMagneticFluxDensity();
        final NEDFrame nedFrame = new NEDFrame();
        final NEDMagneticFluxDensity earthB = new NEDMagneticFluxDensity();
        final CoordinateTransformation cbn = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final CoordinateTransformation cnb = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);

        final int numMeasurements = mMeasurements.size();
        final Matrix x = new Matrix(numMeasurements,
                BodyMagneticFluxDensity.COMPONENTS);
        final Matrix y = new Matrix(numMeasurements,
                BodyMagneticFluxDensity.COMPONENTS);
        final double[] specificForceStandardDeviations = new double[
                numMeasurements];
        int i = 0;
        for (final StandardDeviationFrameBodyMagneticFluxDensity measurement : mMeasurements) {
            final BodyMagneticFluxDensity measuredMagneticFluxDensity =
                    measurement.getMagneticFluxDensity();

            // estimate Earth magnetic flux density at frame position and
            // timestamp using WMM
            final ECEFFrame ecefFrame = measurement.getFrame();
            ECEFtoNEDFrameConverter.convertECEFtoNED(ecefFrame, nedFrame);

            final double year = measurement.getYear();

            final double latitude = nedFrame.getLatitude();
            final double longitude = nedFrame.getLongitude();
            final double height = nedFrame.getHeight();

            nedFrame.getCoordinateTransformation(cbn);
            cbn.inverse(cnb);

            wmmEstimator.estimate(latitude, longitude, height, year, earthB);

            // estimate expected body mangetic flux density taking into
            // account body attitude (inverse of frame orientation) and
            // estimated Earth magnetic flux density
            BodyMagneticFluxDensityEstimator.estimate(earthB, cnb,
                    expectedMagneticFluxDensity);

            final double bMeasX = measuredMagneticFluxDensity.getBx();
            final double bMeasY = measuredMagneticFluxDensity.getBy();
            final double bMeasZ = measuredMagneticFluxDensity.getBz();

            final double bTrueX = expectedMagneticFluxDensity.getBx();
            final double bTrueY = expectedMagneticFluxDensity.getBy();
            final double bTrueZ = expectedMagneticFluxDensity.getBz();

            x.setElementAt(i, 0, bTrueX);
            x.setElementAt(i, 1, bTrueY);
            x.setElementAt(i, 2, bTrueZ);

            y.setElementAt(i, 0, bMeasX);
            y.setElementAt(i, 1, bMeasY);
            y.setElementAt(i, 2, bMeasZ);

            specificForceStandardDeviations[i] =
                    measurement.getMagneticFluxDensityStandardDeviation();
            i++;
        }

        mFitter.setInputData(x, y, specificForceStandardDeviations);
    }
}
