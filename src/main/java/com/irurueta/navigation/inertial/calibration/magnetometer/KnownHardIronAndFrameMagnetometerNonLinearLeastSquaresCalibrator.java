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
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.io.IOException;
import java.util.Collection;

/**
 * Estimates magnetometer soft-iron cross couplings and scaling factors.
 * <p>
 * This calibrator uses an iterative approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 3 measurements at different known frames must
 * be provided. In other words, magnetometer samples must be obtained at 3
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
public class KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator implements
        KnownHardIronAndFrameMagnetometerCalibrator<StandardDeviationFrameBodyMagneticFluxDensity,
                KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener>,
        MagnetometerNonLinearCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 3;

    /**
     * Number of unknowns when common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 6;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 9;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter mFitter =
            new LevenbergMarquardtMultiVariateFitter();

    /**
     * X-coordinate of known hard-iron bias.
     * This is expressed in Teslas (T).
     */
    private double mHardIronX;

    /**
     * Y-coordinate of known hard-iron bias.
     * This is expressed in Teslas (T).
     */
    private double mHardIronY;

    /**
     * Z-coordinate of known hard-iron bias.
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
    private KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener mListener;

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
     * Estimated mean square error respect to provided measurements.
     */
    private double mEstimatedMse;

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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param hardIronY y-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param hardIronZ z-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double hardIronX, final double hardIronY,
            final double hardIronZ) {
        try {
            setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param hardIronY y-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param hardIronZ z-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param listener  listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double hardIronX, final double hardIronY,
            final double hardIronZ,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(hardIronX, hardIronY, hardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIronX    x-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronY    y-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronZ    z-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double hardIronX, final double hardIronY,
            final double hardIronZ) {
        this(hardIronX, hardIronY, hardIronZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIronX    x-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronY    y-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronZ    z-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double hardIronX, final double hardIronY,
            final double hardIronZ,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, hardIronX, hardIronY, hardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ) {
        this(hardIronX, hardIronY, hardIronZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, hardIronX, hardIronY, hardIronZ);
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
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ) {
        this(commonAxisUsed, hardIronX, hardIronY, hardIronZ);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIronX, hardIronY, hardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ) {
        this(hardIronX, hardIronY, hardIronZ);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, hardIronX, hardIronY, hardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ) {
        this(hardIronX, hardIronY, hardIronZ);
        mMeasurements = measurements;
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
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, hardIronX, hardIronY, hardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX,
            final double hardIronY,
            final double hardIronZ) {
        this(magneticModel, hardIronX, hardIronY, hardIronZ);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX,
            final double hardIronY,
            final double hardIronZ,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, hardIronX, hardIronY, hardIronZ);
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
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX,
            final double hardIronY,
            final double hardIronZ) {
        this(commonAxisUsed, magneticModel, hardIronX, hardIronY,
                hardIronZ);
        mMeasurements = measurements;
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
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX,
            final double hardIronY,
            final double hardIronZ,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, hardIronX,
                hardIronY, hardIronZ);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param hardIronY y-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param hardIronZ z-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(hardIronX, hardIronY, hardIronZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param hardIronY y-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param hardIronZ z-coordinate of magnetometer hard-iron bias
     *                  expressed in Teslas (T).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(hardIronX, hardIronY, hardIronZ, initialSx, initialSy,
                initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIronX    x-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronY    y-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronZ    z-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(hardIronX, hardIronY, hardIronZ, initialSx, initialSy,
                initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIronX    x-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronY    y-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronZ    z-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, hardIronX, hardIronY, hardIronZ, initialSx,
                initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(hardIronX, hardIronY, hardIronZ, initialSx, initialSy,
                initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, hardIronX, hardIronY, hardIronZ, initialSx,
                initialSy, initialSz);
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
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(commonAxisUsed, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param initialSx     initial x scaling factor.
     * @param initialSy     initial y scaling factor.
     * @param initialSz     initial z scaling factor.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(hardIronX, hardIronY, hardIronZ, initialSx, initialSy,
                initialSz);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param initialSx     initial x scaling factor.
     * @param initialSy     initial y scaling factor.
     * @param initialSz     initial z scaling factor.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, hardIronX, hardIronY, hardIronZ, initialSx,
                initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param initialSx     initial x scaling factor.
     * @param initialSy     initial y scaling factor.
     * @param initialSz     initial z scaling factor.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(magneticModel, hardIronX, hardIronY, hardIronZ, initialSx,
                initialSy, initialSz);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param initialSx     initial x scaling factor.
     * @param initialSy     initial y scaling factor.
     * @param initialSz     initial z scaling factor.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(magneticModel, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz);
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
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(commonAxisUsed, magneticModel, hardIronX, hardIronY,
                hardIronZ, initialSx, initialSy, initialSz);
        mMeasurements = measurements;
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
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, hardIronX,
                hardIronY, hardIronZ, initialSx, initialSy, initialSz);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param hardIronX  x-coordinate of magnetometer hard-iron bias
     *                   expressed in Teslas (T).
     * @param hardIronY  y-coordinate of magnetometer hard-iron bias
     *                   expressed in Teslas (T).
     * @param hardIronZ  z-coordinate of magnetometer hard-iron bias
     *                   expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(hardIronX, hardIronY, hardIronZ);
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
     * @param hardIronX  x-coordinate of magnetometer hard-iron bias
     *                   expressed in Teslas (T).
     * @param hardIronY  y-coordinate of magnetometer hard-iron bias
     *                   expressed in Teslas (T).
     * @param hardIronZ  z-coordinate of magnetometer hard-iron bias
     *                   expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIronX    x-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronY    y-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronZ    z-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIronX    x-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronY    y-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
     * @param hardIronZ    z-coordinate of magnetometer hard-iron bias
     *                     expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
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
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param initialSx     initial x scaling factor.
     * @param initialSy     initial y scaling factor.
     * @param initialSz     initial z scaling factor.
     * @param initialMxy    initial x-y cross coupling error.
     * @param initialMxz    initial x-z cross coupling error.
     * @param initialMyx    initial y-x cross coupling error.
     * @param initialMyz    initial y-z cross coupling error.
     * @param initialMzx    initial z-x cross coupling error.
     * @param initialMzy    initial z-y cross coupling error.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param initialSx     initial x scaling factor.
     * @param initialSy     initial y scaling factor.
     * @param initialSz     initial z scaling factor.
     * @param initialMxy    initial x-y cross coupling error.
     * @param initialMxz    initial x-z cross coupling error.
     * @param initialMyx    initial y-x cross coupling error.
     * @param initialMyz    initial y-z cross coupling error.
     * @param initialMzx    initial z-x cross coupling error.
     * @param initialMzy    initial z-y cross coupling error.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param initialSx     initial x scaling factor.
     * @param initialSy     initial y scaling factor.
     * @param initialSz     initial z scaling factor.
     * @param initialMxy    initial x-y cross coupling error.
     * @param initialMxz    initial x-z cross coupling error.
     * @param initialMyx    initial y-x cross coupling error.
     * @param initialMyz    initial y-z cross coupling error.
     * @param initialMzx    initial z-x cross coupling error.
     * @param initialMzy    initial z-y cross coupling error.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(magneticModel, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIronX     x-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronY     y-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param hardIronZ     z-coordinate of magnetometer hard-iron bias
     *                      expressed in Teslas (T).
     * @param initialSx     initial x scaling factor.
     * @param initialSy     initial y scaling factor.
     * @param initialSz     initial z scaling factor.
     * @param initialMxy    initial x-y cross coupling error.
     * @param initialMxz    initial x-z cross coupling error.
     * @param initialMyx    initial y-x cross coupling error.
     * @param initialMyz    initial y-z cross coupling error.
     * @param initialMzx    initial z-x cross coupling error.
     * @param initialMzy    initial z-y cross coupling error.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(magneticModel, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, hardIronX, hardIronY,
                hardIronZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
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
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, magneticModel, hardIronX, hardIronY, hardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        mMeasurements = measurements;
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
     * @param hardIronX      x-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronY      y-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
     * @param hardIronZ      z-coordinate of magnetometer hard-iron bias
     *                       expressed in Teslas (T).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double hardIronX, final double hardIronY,
            final double hardIronZ, final double initialSx,
            final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, hardIronX,
                hardIronY, hardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param hardIron known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double[] hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double[] hardIron) {
        this(hardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double[] hardIron) {
        this(hardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final double[] hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, hardIron);
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
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double[] hardIron) {
        this(commonAxisUsed, hardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final double[] hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double[] hardIron) {
        this(hardIron);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final double[] hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double[] hardIron) {
        this(magneticModel, hardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final double[] hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double[] hardIron) {
        this(magneticModel, hardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double[] hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, hardIron);
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
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double[] hardIron) {
        this(commonAxisUsed, magneticModel, hardIron);
        mMeasurements = measurements;
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
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final double[] hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param hardIron known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
     * @param hardIron known hard-iron.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Matrix hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix hardIron) {
        this(hardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final Matrix hardIron) {
        this(hardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final Matrix hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, hardIron);
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
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final Matrix hardIron) {
        this(commonAxisUsed, hardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final Matrix hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final Matrix hardIron) {
        this(hardIron);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final Matrix hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron) {
        this(magneticModel, hardIron);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, hardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron) {
        this(magneticModel, hardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, hardIron);
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
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron) {
        this(commonAxisUsed, magneticModel, hardIron);
        mMeasurements = measurements;
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
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, hardIron);
        mListener = listener;
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
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
     * @param hardIron  known hard-iron.
     * @param initialMm initial soft-iron matrix containing scale factors
     *                  and cross coupling errors.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Matrix hardIron, final Matrix initialMm,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(hardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm) {
        this(hardIron, initialMm);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, hardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        this(hardIron, initialMm);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, hardIron, initialMm);
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
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        this(commonAxisUsed, hardIron, initialMm);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final Matrix hardIron, final Matrix initialMm) {
        this(hardIron, initialMm);
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final Matrix hardIron, final Matrix initialMm,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, hardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron, final Matrix initialMm) {
        this(magneticModel, hardIron, initialMm);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron, final Matrix initialMm,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, hardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron, final Matrix initialMm) {
        this(magneticModel, hardIron, initialMm);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron, final Matrix initialMm,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, hardIron, initialMm);
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
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron, final Matrix initialMm) {
        this(commonAxisUsed, magneticModel, hardIron, initialMm);
        mMeasurements = measurements;
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
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final Matrix hardIron, final Matrix initialMm,
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, hardIron,
                initialMm);
        mListener = listener;
    }

    /**
     * Gets x-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return x-coordinate of known magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronX() {
        return mHardIronX;
    }

    /**
     * Sets x-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronX x coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronX(
            final double hardIronX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronX = hardIronX;
    }

    /**
     * Gets y-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return y-coordinate of known magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronY() {
        return mHardIronY;
    }

    /**
     * Sets y-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronY y coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronY(
            final double hardIronY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronY = hardIronY;
    }

    /**
     * Gets z-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return z-coordinate of known magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronZ() {
        return mHardIronZ;
    }

    /**
     * Sets z-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronZ z coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronZ(
            final double hardIronZ) throws LockedException {
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
    @Override
    public MagneticFluxDensity getHardIronXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mHardIronX,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known x coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    @Override
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
    @Override
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
    @Override
    public MagneticFluxDensity getHardIronYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mHardIronY,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known y coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    @Override
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
    @Override
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
    @Override
    public MagneticFluxDensity getHardIronZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mHardIronZ,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known z coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    @Override
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
    @Override
    public void setHardIronZ(final MagneticFluxDensity hardIronZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mHardIronZ = convertMagneticFluxDensity(hardIronZ);
    }

    /**
     * Sets known hard-iron bias coordinates of magnetometer expressed
     * in Teslas (T).
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron.
     * @param hardIronY y-coordinate of magnetometer hard-iron.
     * @param hardIronZ z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronCoordinates(
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
    @Override
    public void setHardIronCoordinates(
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
    @Override
    public MagneticFluxDensityTriad getHardIronAsTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                mHardIronX, mHardIronY, mHardIronZ);
    }

    /**
     * Gets known hard-iron.
     *
     * @param result instance where result will be stored.
     */
    @Override
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
    @Override
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
    public void setInitialSx(
            final double initialSx) throws LockedException {
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
    public void setInitialSy(
            final double initialSy) throws LockedException {
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
    public void setInitialSz(
            final double initialSz) throws LockedException {
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
    public void setInitialMxy(
            final double initialMxy) throws LockedException {
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
    public void setInitialMxz(
            final double initialMxz) throws LockedException {
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
    public void setInitialMyx(
            final double initialMyx) throws LockedException {
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
    public void setInitialMyz(
            final double initialMyz) throws LockedException {
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
    public void setInitialMzx(
            final double initialMzx) throws LockedException {
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
    public void setInitialMzy(
            final double initialMzy) throws LockedException {
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
            final double initialSz) throws LockedException {
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
            final double initialMzy) throws LockedException {
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
            final double initialMzy) throws LockedException {
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
     * @return array containing coordinates of initial bias.
     */
    @Override
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
    @Override
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
     * @param hardIron known hard-iron bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    @Override
    public void setHardIron(final double[] hardIron) throws LockedException {
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
     * @return hard-iron bias as a column matrix.
     */
    @Override
    public Matrix getHardIronMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                    1);
            getHardIronMatrix(result);
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
    @Override
    public void getHardIronMatrix(final Matrix result) {
        if (result.getRows() != BodyMagneticFluxDensity.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mHardIronX);
        result.setElementAtIndex(1, mHardIronY);
        result.setElementAtIndex(2, mHardIronZ);
    }

    /**
     * Sets known hard-iron bias.
     *
     * @param hardIron magnetometer hard-iron bias to be set.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setHardIron(final Matrix hardIron) throws LockedException {
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
            final Collection<? extends StandardDeviationFrameBodyMagneticFluxDensity> measurements) throws LockedException {
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
    public void setCommonAxisUsed(
            final boolean commonAxisUsed) throws LockedException {
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
    public KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener getListener() {
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
            final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener)
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
        return mMeasurements != null
                && mMeasurements.size() >= MINIMUM_MEASUREMENTS;
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
    public void setMagneticModel(
            final WorldMagneticModel magneticModel)
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
     * Gets estimated covariance matrix for estimated calibration parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy.
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
    @Override
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

        // Where the unknowns are: sx, sy, sz, mxy mxz, myz
        // Reordering:
        //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy = by + mBtruey + sy * mBtruey + myz * mBtruez
        //  mBmeasz = bz + mBtruez + sz * mBtruez

        //  mBmeasx - mBtruex - bx = sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy - mBtruey - by = sy * mBtruey + myz * mBtruez
        //  mBmeasz - mBtruez - bz = sz * mBtruez

        // [mBtruex  0        0        mBtruey  mBtruez  0      ][sx ] = [mBmeasx - mBtruex - bx]
        // [0        mBtruey  0        0        0        mBtruez][sy ]   [mBmeasy - mBtruey - by]
        // [0        0        mBtruez  0        0        0      ][sz ]   [mBmeasz - mBtruez - bz]
        //                                                       [mxy]
        //                                                       [mxz]
        //                                                       [myz]

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiVariateFunctionEvaluator() {
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
                        // We know that:
                        //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
                        //  mBmeasy = by + mBtruey + sy * mBtruey + myz * mBtruez
                        //  mBmeasz = bz + mBtruez + sz * mBtruez

                        // Hence, the derivatives respect the parameters sx, sy, sz,
                        // mxy, mxz, myz

                        // d(fmeasx)/d(sx) = mBtruex
                        // d(fmeasx)/d(sy) = 0.0
                        // d(fmeasx)/d(sz) = 0.0
                        // d(fmeasx)/d(mxy) = mBtruey
                        // d(fmeasx)/d(mxz) = mBtruez
                        // d(fmeasx)/d(myz) = 0.0

                        // d(fmeasy)/d(sx) = 0.0
                        // d(fmeasy)/d(sy) = mBtruey
                        // d(fmeasy)/d(sz) = 0.0
                        // d(fmeasy)/d(mxy) = 0.0
                        // d(fmeasy)/d(mxz) = 0.0
                        // d(fmeasy)/d(myz) = mBtruez

                        // d(fmeasz)/d(sx) = 0.0
                        // d(fmeasz)/d(sy) = 0.0
                        // d(fmeasz)/d(sz) = mBtruez
                        // d(fmeasz)/d(mxy) = 0.0
                        // d(fmeasz)/d(mxz) = 0.0
                        // d(fmeasz)/d(myz) = 0.0

                        final double sx = params[0];
                        final double sy = params[1];
                        final double sz = params[2];

                        final double mxy = params[3];
                        final double mxz = params[4];
                        final double myz = params[5];

                        final double btruex = point[0];
                        final double btruey = point[1];
                        final double btruez = point[2];

                        result[0] = mHardIronX + btruex + sx * btruex + mxy * btruey + mxz * btruez;
                        result[1] = mHardIronY + btruey + sy * btruey + myz * btruez;
                        result[2] = mHardIronZ + btruez + sz * btruez;

                        jacobian.setElementAt(0, 0, btruex);
                        jacobian.setElementAt(0, 1, 0.0);
                        jacobian.setElementAt(0, 2, 0.0);
                        jacobian.setElementAt(0, 3, btruey);
                        jacobian.setElementAt(0, 4, btruez);
                        jacobian.setElementAt(0, 5, 0.0);

                        jacobian.setElementAt(1, 0, 0.0);
                        jacobian.setElementAt(1, 1, btruey);
                        jacobian.setElementAt(1, 2, 0.0);
                        jacobian.setElementAt(1, 3, 0.0);
                        jacobian.setElementAt(1, 4, 0.0);
                        jacobian.setElementAt(1, 5, btruez);

                        jacobian.setElementAt(2, 0, 0.0);
                        jacobian.setElementAt(2, 1, 0.0);
                        jacobian.setElementAt(2, 2, btruez);
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
        mEstimatedMse = mFitter.getMse();
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
        // The accelerometer model is:
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

        //  mBmeasx - mBtruex - bx = sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        //  mBmeasy - mBtruey - by = myx * mBtruex + sy * mBtruey + myz * mBtruez
        //  mBmeasz - mBtruez - bz = mzx * mBtruex + mzy * mBtruey + sz * mBtruez

        // [mBtruex  0        0        mBtruey  mBtruez  0        0        0        0      ][sx ] = [mBmeasx - mBtruex - bx]
        // [0        mBtruey  0        0        0        mBtruex  mBtruez  0        0      ][sy ]   [mBmeasy - mBtruey - by]
        // [0        0        mBtruez  0        0        0        0        mBtruex  mBtruey][sz ]   [mBmeasz - mBtruez - bz]
        //                                                                                  [mxy]
        //                                                                                  [mxz]
        //                                                                                  [myx]
        //                                                                                  [myz]
        //                                                                                  [mzx]
        //                                                                                  [mzy]

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiVariateFunctionEvaluator() {
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
                            final int i, final double[] point,
                            final double[] result, final double[] params,
                            final Matrix jacobian) {
                        // We know that:
                        //  mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
                        //  mBmeasy = by + myx * mBtruex + mBtruey + sy * mBtruey + myz * mBtruez
                        //  mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + mBtruez + sz * mBtruez

                        // Hence, the derivatives respect the parameters sx, sy, sz,
                        // mxy, mxz, myx, myz, mzx and mzy is:

                        // d(fmeasx)/d(sx) = mBtruex
                        // d(fmeasx)/d(sy) = 0.0
                        // d(fmeasx)/d(sz) = 0.0
                        // d(fmeasx)/d(mxy) = mBtruey
                        // d(fmeasx)/d(mxz) = mBtruez
                        // d(fmeasx)/d(myx) = 0.0
                        // d(fmeasx)/d(myz) = 0.0
                        // d(fmeasx)/d(mzx) = 0.0
                        // d(fmeasx)/d(mzy) = 0.0

                        // d(fmeasy)/d(sx) = 0.0
                        // d(fmeasy)/d(sy) = mBtruey
                        // d(fmeasy)/d(sz) = 0.0
                        // d(fmeasy)/d(mxy) = 0.0
                        // d(fmeasy)/d(mxz) = 0.0
                        // d(fmeasy)/d(myx) = mBtruex
                        // d(fmeasy)/d(myz) = mBtruez
                        // d(fmeasy)/d(mzx) = 0.0
                        // d(fmeasy)/d(mzy) = 0.0

                        // d(fmeasz)/d(sx) = 0.0
                        // d(fmeasz)/d(sy) = 0.0
                        // d(fmeasz)/d(sz) = mBtruez
                        // d(fmeasz)/d(mxy) = 0.0
                        // d(fmeasz)/d(mxz) = 0.0
                        // d(fmeasz)/d(myx) = 0.0
                        // d(fmeasz)/d(myz) = 0.0
                        // d(fmeasz)/d(mzx) = mBtruex
                        // d(fmeasz)/d(mzy) = mBtruey

                        final double sx = params[0];
                        final double sy = params[1];
                        final double sz = params[2];

                        final double mxy = params[3];
                        final double mxz = params[4];
                        final double myx = params[5];
                        final double myz = params[6];
                        final double mzx = params[7];
                        final double mzy = params[8];

                        final double btruex = point[0];
                        final double btruey = point[1];
                        final double btruez = point[2];

                        result[0] = mHardIronX + btruex + sx * btruex + mxy * btruey + mxz * btruez;
                        result[1] = mHardIronY + myx * btruex + btruey + sy * btruey + myz * btruez;
                        result[2] = mHardIronZ + mzx * btruex + mzy * btruey + btruez + sz * btruez;

                        jacobian.setElementAt(0, 0, btruex);
                        jacobian.setElementAt(0, 1, 0.0);
                        jacobian.setElementAt(0, 2, 0.0);
                        jacobian.setElementAt(0, 3, btruey);
                        jacobian.setElementAt(0, 4, btruez);
                        jacobian.setElementAt(0, 5, 0.0);
                        jacobian.setElementAt(0, 6, 0.0);
                        jacobian.setElementAt(0, 7, 0.0);
                        jacobian.setElementAt(0, 8, 0.0);

                        jacobian.setElementAt(1, 0, 0.0);
                        jacobian.setElementAt(1, 1, btruey);
                        jacobian.setElementAt(1, 2, 0.0);
                        jacobian.setElementAt(1, 3, 0.0);
                        jacobian.setElementAt(1, 4, 0.0);
                        jacobian.setElementAt(1, 5, btruex);
                        jacobian.setElementAt(1, 6, btruez);
                        jacobian.setElementAt(1, 7, 0.0);
                        jacobian.setElementAt(1, 8, 0.0);

                        jacobian.setElementAt(2, 0, 0.0);
                        jacobian.setElementAt(2, 1, 0.0);
                        jacobian.setElementAt(2, 2, btruez);
                        jacobian.setElementAt(2, 3, 0.0);
                        jacobian.setElementAt(2, 4, 0.0);
                        jacobian.setElementAt(2, 5, 0.0);
                        jacobian.setElementAt(2, 6, 0.0);
                        jacobian.setElementAt(2, 7, btruex);
                        jacobian.setElementAt(2, 8, btruey);
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
        mEstimatedMse = mFitter.getMse();
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
