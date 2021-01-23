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
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.io.IOException;
import java.util.Collection;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.List;

/**
 * Estimates magnetometer hard-iron biases, cross couplings and scaling
 * factors.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared
 * error solution.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single known
 * position and instant must be taken at 10 different unknown orientations and
 * zero velocity when common z-axis is assumed, otherwise at least 13
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
public class KnownPositionAndInstantMagnetometerCalibrator implements
        MagnetometerNonLinearCalibrator, UnknownHardIronNonLinearMagnetometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

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
     * Required minimum number of measurements when common z-axis is assumed.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS = COMMON_Z_AXIS_UNKNOWNS + 1;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL = GENERAL_UNKNOWNS + 1;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter mFitter =
            new LevenbergMarquardtMultiDimensionFitter();

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
     * Contains a collection of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     */
    private Collection<StandardDeviationBodyMagneticFluxDensity> mMeasurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from Mm matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownPositionAndInstantMagnetometerCalibratorListener mListener;

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
     * Internally holds x-coordinate of measured magnetic flux density
     * during calibration.
     */
    private double mBmeasX;

    /**
     * Internally holds y-coordinate of measured magnetic flux density
     * during calibration.
     */
    private double mBmeasY;

    /**
     * Internally holds z-coordinate of measured magnetic flux density
     * during calibration.
     */
    private double mBmeasZ;

    /**
     * Internaly holds measured magnetic flux density during calibration
     * expressed as a column matrix.
     */
    private Matrix mBmeas;

    /**
     * Internally holds cross-coupling errors during calibration.
     */
    private Matrix mM;

    /**
     * Internally holds inverse of cross-coupling errors during calibration.
     */
    private Matrix mInvM;

    /**
     * Internally holds biases during calibration.
     */
    private Matrix mB;

    /**
     * Internally holds computed true magnetic flux density during
     * calibration.
     */
    private Matrix mBtrue;

    /**
     * Constructor.
     */
    public KnownPositionAndInstantMagnetometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final WorldMagneticModel magneticModel) {
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
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
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
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
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
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
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
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
    public KnownPositionAndInstantMagnetometerCalibrator(
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
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
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
    public KnownPositionAndInstantMagnetometerCalibrator(
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
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        this(initialHardIron);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        this(position, measurements, initialHardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        this(initialHardIron);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        this(position, measurements, initialHardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron, initialMm);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final Matrix initialMm) {
        this(position, measurements, initialHardIron, initialMm);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
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
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final Matrix initialMm,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, initialHardIron,
                initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
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
    public KnownPositionAndInstantMagnetometerCalibrator(
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
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
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
    public KnownPositionAndInstantMagnetometerCalibrator(
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
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        this(convertPosition(position), measurements,
                initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        this(convertPosition(position), measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(convertPosition(position), measurements, initialHardIron,
                initialMm);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron,
                initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final Matrix initialMm) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
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
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final Matrix initialMm,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed,
                initialHardIron, initialMm, listener);
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
     * Gets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial x-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mInitialHardIronX,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronXAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(mInitialHardIronX);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronX(final MagneticFluxDensity initialHardIronX)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronX = convertMagneticFluxDensity(initialHardIronX);
    }

    /**
     * Gets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial y-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mInitialHardIronY,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronYAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(mInitialHardIronY);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronY initial y-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronY(final MagneticFluxDensity initialHardIronY)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronY = convertMagneticFluxDensity(initialHardIronY);
    }

    /**
     * Gets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial z-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mInitialHardIronZ,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronZAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(mInitialHardIronZ);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronZ initial z-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronZ(final MagneticFluxDensity initialHardIronZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronZ = convertMagneticFluxDensity(initialHardIronZ);
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
            final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronX = initialHardIronX;
        mInitialHardIronY = initialHardIronY;
        mInitialHardIronZ = initialHardIronZ;
    }

    /**
     * Sets initial hard iron coordinates of magnetometer used to find a solution.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer bias.
     * @param initialHardIronY initial y-coordinate of magnetometer bias.
     * @param initialHardIronZ initial z-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIron(
            final MagneticFluxDensity initialHardIronX,
            final MagneticFluxDensity initialHardIronY,
            final MagneticFluxDensity initialHardIronZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInitialHardIronX = convertMagneticFluxDensity(initialHardIronX);
        mInitialHardIronY = convertMagneticFluxDensity(initialHardIronY);
        mInitialHardIronZ = convertMagneticFluxDensity(initialHardIronZ);
    }

    /**
     * Gets initial hard-iron used to find a solution.
     *
     * @return initial hard-iron.
     */
    @Override
    public MagneticFluxDensityTriad getInitialHardIronAsTriad() {
        return new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.TESLA,
                mInitialHardIronX, mInitialHardIronY, mInitialHardIronZ);
    }

    /**
     * Gets initial hard-iron used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronAsTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(
                mInitialHardIronX, mInitialHardIronY, mInitialHardIronZ,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets initial hard-iron used to find a solution.
     *
     * @param initialHardIron initial hard-iron to be set.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIron(final MagneticFluxDensityTriad initialHardIron)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInitialHardIronX = convertMagneticFluxDensity(
                initialHardIron.getValueX(), initialHardIron.getUnit());
        mInitialHardIronY = convertMagneticFluxDensity(
                initialHardIron.getValueY(), initialHardIron.getUnit());
        mInitialHardIronZ = convertMagneticFluxDensity(
                initialHardIron.getValueZ(), initialHardIron.getUnit());
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
     * Gets initial hard-iron  bias to be used to find a solution as an array.
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
     * Values are expressed in Teslas (T).
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
     * Values are expressed in Teslas (T).
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
     * Sets initial hard-iron bias to be used to find a solution as a column
     * matrix with values expressed in Teslas (T).
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
    public Collection<StandardDeviationBodyMagneticFluxDensity> getMeasurements() {
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
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements)
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
    public KnownPositionAndInstantMagnetometerCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final KnownPositionAndInstantMagnetometerCalibratorListener listener)
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
    @Override
    public boolean isReady() {
        return mMeasurements != null && mMeasurements.size() >= getMinimumRequiredMeasurements()
                && mPosition != null && mYear != null;
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

        } catch (final AlgebraException | FittingException
                | com.irurueta.numerical.NotReadyException
                | IOException e) {
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
     * Gets x coordinate of estimated magnetometer bias.
     *
     * @return x coordinate of estimated magnetometer bias.
     */
    @Override
    public MagneticFluxDensity getEstimatedHardIronXAsMagneticFluxDensity() {
        return mEstimatedHardIron != null ?
                new MagneticFluxDensity(mEstimatedHardIron[0], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets x coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronXAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedHardIron != null) {
            result.setValue(mEstimatedHardIron[0]);
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets y coordinate of estimated magnetometer bias.
     *
     * @return y coordinate of estimated magnetometer bias.
     */
    @Override
    public MagneticFluxDensity getEstimatedHardIronYAsMagneticFluxDensity() {
        return mEstimatedHardIron != null ?
                new MagneticFluxDensity(mEstimatedHardIron[1], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets y coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronYAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedHardIron != null) {
            result.setValue(mEstimatedHardIron[1]);
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets z coordinate of estimated magnetometer bias.
     *
     * @return z coordinate of estimated magnetometer bias.
     */
    @Override
    public MagneticFluxDensity getEstimatedHardIronZAsMagneticFluxDensity() {
        return mEstimatedHardIron != null ?
                new MagneticFluxDensity(mEstimatedHardIron[2], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets z coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronZAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedHardIron != null) {
            result.setValue(mEstimatedHardIron[2]);
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated magnetometer bias.
     *
     * @return estimated magnetometer bias or null if not available.
     */
    @Override
    public MagneticFluxDensityTriad getEstimatedHardIronAsTriad() {
        return mEstimatedHardIron != null ?
                new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                        mEstimatedHardIron[0], mEstimatedHardIron[1], mEstimatedHardIron[2]) : null;
    }

    /**
     * Gets estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available and result was
     * modified, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronAsTriad(final MagneticFluxDensityTriad result) {
        if (mEstimatedHardIron != null) {
            result.setValueCoordinatesAndUnit(
                    mEstimatedHardIron[0], mEstimatedHardIron[1], mEstimatedHardIron[2],
                    MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
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
     * parameters (following indicated order): bx, by, bz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy.
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
     * Gets estimated mean square error respect to provided measurements.
     *
     * @return estimated mean square error respect to provided measurements.
     */
    @Override
    public double getEstimatedMse() {
        return mEstimatedMse;
    }

    /**
     * Gets variance of estimated x coordinate of magnetometer bias expressed in
     * squared Teslas (T^2).
     *
     * @return variance of estimated x coordinate of magnetometer bias or null if
     * not available.
     */
    public Double getEstimatedHardIronXVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated x coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronXStandardDeviation() {
        final Double variance = getEstimatedHardIronXVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated x coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronXStandardDeviation(),
                        MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated x coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronXStandardDeviation());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated y coordinate of magnetometer bias expressed in
     * squared Teslas (T^2).
     *
     * @return variance of estimated y coordinate of magnetometer bias or null if
     * not available.
     */
    public Double getEstimatedHardIronYVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated y coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronYStandardDeviation() {
        final Double variance = getEstimatedHardIronYVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated y coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronYStandardDeviation(),
                        MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated y coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronYStandardDeviation());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated z coordinate of magnetometer bias expressed in
     * squared Teslas (T^2).
     *
     * @return variance of estimated yzcoordinate of magnetometer bias or null if
     * not available.
     */
    public Double getEstimatedHardIronZVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated z coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronZStandardDeviation() {
        final Double variance = getEstimatedHardIronZVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated z coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronZStandardDeviation(),
                        MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated z coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronZStandardDeviation());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets standard deviation of estimated magnetometer bias coordinates.
     *
     * @return standard deviation of estimated magnetometer bias coordinates.
     */
    public MagneticFluxDensityTriad getEstimatedHardIronStandardDeviation() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                        getEstimatedHardIronXStandardDeviation(),
                        getEstimatedHardIronYStandardDeviation(),
                        getEstimatedHardIronZStandardDeviation()) : null;
    }

    /**
     * Gets standard deviation of estimated magnetometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of magnetometer bias was available,
     * false otherwise.
     */
    public boolean getEstimatedHardIronStandardDeviation(final MagneticFluxDensityTriad result) {
        if (mEstimatedCovariance != null) {
            result.setValueCoordinatesAndUnit(
                    getEstimatedHardIronXStandardDeviation(),
                    getEstimatedHardIronYStandardDeviation(),
                    getEstimatedHardIronZStandardDeviation(),
                    MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets average of estimated standard deviation of magnetometer bias coordinates
     * expressed in Teslas (T).
     *
     * @return average of estimated standard deviation of magnetometer bias coordinates,
     * or null if not available.
     */
    public Double getEstimatedHardIronStandardDeviationAverage() {
        return mEstimatedCovariance != null ?
                (getEstimatedHardIronXStandardDeviation() +
                        getEstimatedHardIronYStandardDeviation() +
                        getEstimatedHardIronZStandardDeviation()) / 3.0 : null;
    }

    /**
     * Gets average of estimated standard deviation of magnetometer bias coordinates.
     *
     * @return average of estimated standard deviation of magnetometer bias coordinates,
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronStandardDeviationAverage(),
                        MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets average of estimated standard deviation of magnetometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if average of estimated standard deviation of magnetometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronStandardDeviationAverage());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets norm of estimated standard deviation of magnetometer bias expressed in
     * Teslas (T).
     *
     * @return norm of estimated standard deviation of magnetometer bias or null
     * if not available.
     */
    public Double getEstimatedHardIronStandardDeviationNorm() {
        return mEstimatedCovariance != null ?
                Math.sqrt(getEstimatedHardIronXVariance()
                        + getEstimatedHardIronYVariance()
                        + getEstimatedHardIronZVariance()) : null;
    }

    /**
     * Gets norm of estimated standard deviation of magnetometer bias.
     *
     * @return norm of estimated standard deviation of magnetometer bias or null
     * if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronStandardDeviationNorm(),
                        MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets norm of estimated standard deviation of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if norm of estimated standard deviation of magnetometer bias
     * is available, false otherwise.
     */
    public boolean getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronStandardDeviationNorm());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     * @throws IOException        if world magnetic model cannot be loaded.
     */
    private void setInputData() throws WrongSizeException, IOException {

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator;
        if (mMagneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(mMagneticModel);
        } else {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }

        final NEDPosition position = getNedPosition();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, mYear);
        final double b = earthB.getNorm();
        final double b2 = b * b;

        final int numMeasurements = mMeasurements.size();
        final Matrix x = new Matrix(numMeasurements,
                BodyMagneticFluxDensity.COMPONENTS);
        final double[] y = new double[numMeasurements];
        final double[] specificForceStandardDeviations = new double[numMeasurements];
        int i = 0;
        for (final StandardDeviationBodyMagneticFluxDensity measurement : mMeasurements) {
            final BodyMagneticFluxDensity measuredMagneticFluxDensity =
                    measurement.getMagneticFluxDensity();

            final double bmeasX = measuredMagneticFluxDensity.getBx();
            final double bmeasY = measuredMagneticFluxDensity.getBy();
            final double bmeasZ = measuredMagneticFluxDensity.getBz();

            x.setElementAt(i, 0, bmeasX);
            x.setElementAt(i, 1, bmeasY);
            x.setElementAt(i, 2, bmeasZ);

            y[i] = b2;

            specificForceStandardDeviations[i] =
                    measurement.getMagneticFluxDensityStandardDeviation();

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
     * @throws IOException                              if world magnetic model cannot be loaded.
     */
    private void calibrateGeneral() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException, IOException {
        // The magnetometer model is:
        // bmeas = ba + (I + Mm) * btrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // bmeas = ba + (I + Mm) * btrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // magnetometer model can be better expressed as:
        // bmeas = T*K*(btrue + b)
        // bmeas = M*(btrue + b)
        // bmeas = M*btrue + M*b

        // where:
        // M = I + Mm
        // bm = M*b = (I + Mm)*b --> b = M^-1*bm

        // We know that the norm of the true body magnetic flux density
        // is equal to the amount of Earth magnetic flux density at provided
        // position and timestamp
        // ||btrue|| = ||bEarth|| --> from 30 µT to 60 µT

        // Hence:
        // bmeas - M*b = M*btrue

        // M^-1 * (bmeas - M*b) = btrue

        // ||bEarth||^2 = ||btrue||^2 = (M^-1 * (bmeas - M*b))^T * (M^-1 * (bmeas - M*b))
        // ||bEarth||^2 = (bmeas - M*b)^T*(M^-1)^T * M^-1 * (bmeas - M*b)
        // ||bEarth||^2 = (bmeas - M * b)^T * ||M^-1||^2 * (bmeas - M * b)
        // ||bEarth||^2 = ||bmeas - M * b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [m21 	m22 	m23]
        //     [m31 	m32 	m33]

        final GradientEstimator gradientEstimator = new GradientEstimator(
                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(final double[] point)
                            throws EvaluationException {
                        return evaluateGeneral(point);
                    }
                });

        final Matrix initialM = Matrix.identity(BodyMagneticFluxDensity.COMPONENTS,
                BodyMagneticFluxDensity.COMPONENTS);
        initialM.add(getInitialMm());

        final Matrix invInitialM = Utils.inverse(initialM);
        final Matrix initialBm = getInitialHardIronAsMatrix();
        final Matrix initialB = invInitialM.multiplyAndReturnNew(initialBm);

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Input points are measured magnetic flux density coordinates
                        return BodyMagneticFluxDensity.COMPONENTS;
                    }

                    @Override
                    public double[] createInitialParametersArray() {
                        final double[] initial = new double[GENERAL_UNKNOWNS];

                        // biases b
                        for (int i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
                            initial[i] = initialB.getElementAtIndex(i);
                        }

                        // cross coupling errors M
                        final int num = BodyMagneticFluxDensity.COMPONENTS
                                * BodyMagneticFluxDensity.COMPONENTS;
                        for (int i = 0, j = BodyMagneticFluxDensity.COMPONENTS; i < num; i++, j++) {
                            initial[j] = initialM.getElementAtIndex(i);
                        }

                        return initial;
                    }

                    @Override
                    public double evaluate(
                            final int i, final double[] point,
                            final double[] params, final double[] derivatives)
                            throws EvaluationException {

                        mBmeasX = point[0];
                        mBmeasY = point[1];
                        mBmeasZ = point[2];

                        gradientEstimator.gradient(params, derivatives);

                        return evaluateGeneral(params);
                    }
                });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double bx = result[0];
        final double by = result[1];
        final double bz = result[2];

        final double m11 = result[3];
        final double m21 = result[4];
        final double m31 = result[5];

        final double m12 = result[6];
        final double m22 = result[7];
        final double m32 = result[8];

        final double m13 = result[9];
        final double m23 = result[10];
        final double m33 = result[11];

        final Matrix b = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                1);
        b.setElementAtIndex(0, bx);
        b.setElementAtIndex(1, by);
        b.setElementAtIndex(2, bz);

        final Matrix m = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                BodyMagneticFluxDensity.COMPONENTS);
        m.setElementAtIndex(0, m11);
        m.setElementAtIndex(1, m21);
        m.setElementAtIndex(2, m31);

        m.setElementAtIndex(3, m12);
        m.setElementAtIndex(4, m22);
        m.setElementAtIndex(5, m32);

        m.setElementAtIndex(6, m13);
        m.setElementAtIndex(7, m23);
        m.setElementAtIndex(8, m33);

        setResult(m, b);

        // at this point covariance is expressed in terms of b and M, and must
        // be expressed in terms of ba and Ma.
        // We know that:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [m21 	m22 	m23]
        //     [m31 	m32 	m33]

        // and that ba and Ma are expressed as:
        // Mm = M - I
        // bm = M * b

        // Mm = [m11 - 1    m12         m13    ] =  [sx     mxy     mxz]
        //      [m21        m22 - 1     m23    ]    [myx    sy      myz]
        //      [m31        m32         m33 - 1]    [mzx    mzy     sz ]

        // bm = [m11 * bx + m12 * by + m13 * bz] = 	[bmx]
        //      [m21 * bx + m22 * by + m23 * bz]	[bmy]
        //      [m31 * bx + m32 * by + m33 * bz]	[bmz]

        // Defining the linear application:
        // F(b, M) = F(bx, by, bz, m11, m21, m31, m12, m22, m32, m13, m23, m33)
        // as:
        //[bmx] = 	[m11 * bx + m12 * by + m13 * bz]
        //[bmy]		[m21 * bx + m22 * by + m23 * bz]
        //[bmz]		[m31 * bx + m32 * by + m33 * bz]
        //[sx]		[m11 - 1]
        //[sy]		[m22 - 1]
        //[sz]		[m33 -1]
        //[mxy]	    [m12]
        //[mxz]	    [m13]
        //[myx]	    [m21]
        //[myz]	    [m23]
        //[mzx]	    [m31]
        //[mzy]	    [m32]

        // Then the Jacobian of F(b, M) is:
        // J = 	[m11  m12  m13  bx  0   0   by  0   0   bz  0   0 ]
        //	    [m21  m22  m23  0   bx  0   0   by  0   0   bz  0 ]
        //	    [m31  m32  m33  0   0   bx  0   0   by  0   0   bz]
        //	    [0    0    0    1   0   0   0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   1   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0   0   0   1 ]
        //	    [0    0    0    0   0   0   1   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0   1   0   0 ]
        //	    [0    0    0    0   1   0   0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0   0   1   0 ]
        //	    [0    0    0    0   0   1   0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   1   0   0   0 ]

        // We know that the propagated covariance is J * Cov * J', hence:
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS, GENERAL_UNKNOWNS);

        jacobian.setElementAt(0, 0, m11);
        jacobian.setElementAt(1, 0, m21);
        jacobian.setElementAt(2, 0, m31);

        jacobian.setElementAt(0, 1, m12);
        jacobian.setElementAt(1, 1, m22);
        jacobian.setElementAt(2, 1, m32);

        jacobian.setElementAt(0, 2, m13);
        jacobian.setElementAt(1, 2, m23);
        jacobian.setElementAt(2, 2, m33);

        jacobian.setElementAt(0, 3, bx);
        jacobian.setElementAt(3, 3, 1.0);

        jacobian.setElementAt(1, 4, bx);
        jacobian.setElementAt(8, 4, 1.0);

        jacobian.setElementAt(2, 5, bx);
        jacobian.setElementAt(10, 5, 1.0);

        jacobian.setElementAt(0, 6, by);
        jacobian.setElementAt(6, 6, 1.0);

        jacobian.setElementAt(1, 7, by);
        jacobian.setElementAt(4, 7, 1.0);

        jacobian.setElementAt(2, 8, by);
        jacobian.setElementAt(11, 8, 1.0);

        jacobian.setElementAt(0, 9, bz);
        jacobian.setElementAt(7, 9, 1.0);

        jacobian.setElementAt(1, 10, bz);
        jacobian.setElementAt(9, 10, 1.0);

        jacobian.setElementAt(2, 11, bz);
        jacobian.setElementAt(5, 11, 1.0);

        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws FittingException                         if Levenberg-Marquardt fails for numerical reasons.
     * @throws AlgebraException                         if there are numerical instabilities that prevent
     *                                                  matrix inversion.
     * @throws com.irurueta.numerical.NotReadyException never happens.
     * @throws IOException                              if world magnetic model cannot be loaded.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException, IOException {
        // The magnetometer model is:
        // bmeas = bm + (I + Mm) * btrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // bmeas = bm + (I + Mm) * btrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // magnetometer model can be better expressed as:
        // bmeas = T*K*(btrue + b)
        // bmeas = M*(btrue + b)
        // bmeas = M*btrue + M*b

        //where:
        // M = I + Mm
        // bm = M*b = (I + Mm)*b --> b = M^-1*bm

        // We know that the norm of the true body magnetic flux density
        // is equal to the amount of Earth magnetic flux density at provided
        // position and timestamp
        // ||btrue|| = ||bEarth|| --> from 30 µT to 60 µT

        // Hence:
        // bmeas - M*b = M*btrue

        // M^-1 * (bmeas - M*b) = btrue

        // ||bEarth||^2 = ||btrue||^2 = (M^-1 * (bmeas - M*b))^T * (M^-1 * (bmeas - M*b))
        // ||bEarth||^2 = (bmeas - M*b)^T*(M^-1)^T * M^-1 * (bmeas - M*b)
        // ||bEarth||^2 = (bmeas - M * b)^T * ||M^-1||^2 * (bmeas - M * b)
        // ||bEarth||^2 = ||bmeas - M * b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [0 		m22 	m23]
        //     [0 	 	0 		m33]


        final GradientEstimator gradientEstimator = new GradientEstimator(
                new MultiDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(final double[] point)
                            throws EvaluationException {
                        return evaluateCommonAxis(point);
                    }
                });

        final Matrix initialM = Matrix.identity(
                BodyMagneticFluxDensity.COMPONENTS,
                BodyMagneticFluxDensity.COMPONENTS);
        initialM.add(getInitialMm());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        final Matrix invInitialM = Utils.inverse(initialM);
        final Matrix initialBm = getInitialHardIronAsMatrix();
        final Matrix initialB = invInitialM.multiplyAndReturnNew(initialBm);

        mFitter.setFunctionEvaluator(
                new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                    @Override
                    public int getNumberOfDimensions() {
                        // Input points are measured magnetic flux density coordinates
                        return BodyKinematics.COMPONENTS;
                    }

                    @Override
                    public double[] createInitialParametersArray() {
                        final double[] initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                        // biases b
                        for (int i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
                            initial[i] = initialB.getElementAtIndex(i);
                        }

                        // upper diagonal cross coupling errors M
                        int k = BodyMagneticFluxDensity.COMPONENTS;
                        for (int j = 0; j < BodyMagneticFluxDensity.COMPONENTS; j++) {
                            for (int i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
                                if (i <= j) {
                                    initial[k] = initialM.getElementAt(i, j);
                                    k++;
                                }
                            }
                        }

                        return initial;
                    }

                    @Override
                    public double evaluate(
                            final int i, final double[] point,
                            final double[] params, final double[] derivatives)
                            throws EvaluationException {

                        mBmeasX = point[0];
                        mBmeasY = point[1];
                        mBmeasZ = point[2];

                        gradientEstimator.gradient(params, derivatives);

                        return evaluateCommonAxis(params);
                    }
                });

        setInputData();

        mFitter.fit();

        final double[] result = mFitter.getA();

        final double bx = result[0];
        final double by = result[1];
        final double bz = result[2];

        final double m11 = result[3];

        final double m12 = result[4];
        final double m22 = result[5];

        final double m13 = result[6];
        final double m23 = result[7];
        final double m33 = result[8];

        final Matrix b = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                1);
        b.setElementAtIndex(0, bx);
        b.setElementAtIndex(1, by);
        b.setElementAtIndex(2, bz);

        final Matrix m = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                BodyMagneticFluxDensity.COMPONENTS);
        m.setElementAtIndex(0, m11);
        m.setElementAtIndex(1, 0.0);
        m.setElementAtIndex(2, 0.0);

        m.setElementAtIndex(3, m12);
        m.setElementAtIndex(4, m22);
        m.setElementAtIndex(5, 0.0);

        m.setElementAtIndex(6, m13);
        m.setElementAtIndex(7, m23);
        m.setElementAtIndex(8, m33);

        setResult(m, b);

        // at this point covariance is expressed in terms of b and M, and must
        // be expressed in terms of ba and Ma.
        // We know that:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [0    	m22 	m23]
        //     [0    	0    	m33]

        // m21 = m31 = m32 = 0

        // and that ba and Ma are expressed as:
        // Ma = M - I
        // ba = M * b

        // Ma = [m11 - 1    m12         m13    ] =  [sx     mxy     mxz]
        //      [0          m22 - 1     m23    ]    [0      sy      myz]
        //      [0          0           m33 - 1]    [0      0       sz ]

        // ba = [m11 * bx + m12 * by + m13 * bz] = 	[bax]
        //      [           m22 * by + m23 * bz]	[bay]
        //      [                      m33 * bz]	[baz]

        // Defining the linear application:
        // F(b, M) = F(bx, by, bz, m11, m12, m22, m13, m23, m33)
        // as:
        //[bax] = 	[m11 * bx + m12 * by + m13 * bz]
        //[bay]		[m22 * by + m23 * bz]
        //[baz]		[m33 * bz]
        //[sx]		[m11 - 1]
        //[sy]		[m22 - 1]
        //[sz]		[m33 -1]
        //[mxy]	    [m12]
        //[mxz]	    [m13]
        //[myx]	    [0]
        //[myz]	    [m23]
        //[mzx]	    [0]
        //[mzy]	    [0]

        // Then the Jacobian of F(b, M) is:
        // J = 	[m11  m12  m13  bx  by  0   bz  0   0 ]
        //	    [0    m22  m23  0   0   by  0   bz  0 ]
        //	    [0    0    m33  0   0   0   0   0   bz]
        //	    [0    0    0    1   0   0   0   0   0 ]
        //	    [0    0    0    0   0   1   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   1 ]
        //	    [0    0    0    0   1   0   0   0   0 ]
        //	    [0    0    0    0   0   0   1   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   1   0 ]
        //	    [0    0    0    0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0 ]

        // We know that the propagated covariance is J * Cov * J', hence:
        final Matrix jacobian = new Matrix(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);

        jacobian.setElementAt(0, 0, m11);

        jacobian.setElementAt(0, 1, m12);
        jacobian.setElementAt(1, 1, m22);

        jacobian.setElementAt(0, 2, m13);
        jacobian.setElementAt(1, 2, m23);
        jacobian.setElementAt(2, 2, m33);

        jacobian.setElementAt(0, 3, bx);
        jacobian.setElementAt(3, 3, 1.0);

        jacobian.setElementAt(0, 4, by);
        jacobian.setElementAt(6, 4, 1.0);

        jacobian.setElementAt(1, 5, by);
        jacobian.setElementAt(4, 5, 1.0);

        jacobian.setElementAt(0, 6, bz);
        jacobian.setElementAt(7, 6, 1.0);

        jacobian.setElementAt(1, 7, bz);
        jacobian.setElementAt(9, 7, 1.0);

        jacobian.setElementAt(2, 8, bz);
        jacobian.setElementAt(5, 8, 1.0);

        final Matrix jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(mEstimatedCovariance);
        jacobian.multiply(jacobianTrans);
        mEstimatedCovariance = jacobian;
    }

    /**
     * Makes proper conversion of internal cross-coupling and bias matrices.
     *
     * @param m internal cross-coupling matrix.
     * @param b internal bias matrix.
     * @throws AlgebraException if a numerical instability occurs.
     */
    private void setResult(final Matrix m, final Matrix b) throws AlgebraException {
        // Because:
        // M = I + Mm
        // b = M^-1*bm

        // Then:
        // Mm = M - I
        // bm = M*b

        if (mEstimatedHardIron == null) {
            mEstimatedHardIron = new double[
                    BodyMagneticFluxDensity.COMPONENTS];
        }

        final Matrix bm = m.multiplyAndReturnNew(b);
        bm.toArray(mEstimatedHardIron);

        if (mEstimatedMm == null) {
            mEstimatedMm = m;
        } else {
            mEstimatedMm.copyFrom(m);
        }

        for (int i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
            mEstimatedMm.setElementAt(i, i,
                    mEstimatedMm.getElementAt(i, i) - 1.0);
        }

        mEstimatedCovariance = mFitter.getCovar();
        mEstimatedChiSq = mFitter.getChisq();
        mEstimatedMse = mFitter.getMse();
    }

    /**
     * Computes estimated true magnetic flux density squared norm using current measured
     * body magnetic flux density and provided parameters for the general case.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the general purpose case.
     *               Must have length 12.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateGeneral(final double[] params) throws EvaluationException {
        final double bx = params[0];
        final double by = params[1];
        final double bz = params[2];

        final double m11 = params[3];
        final double m21 = params[4];
        final double m31 = params[5];

        final double m12 = params[6];
        final double m22 = params[7];
        final double m32 = params[8];

        final double m13 = params[9];
        final double m23 = params[10];
        final double m33 = params[11];

        return evaluate(bx, by, bz, m11, m21, m31, m12, m22, m32,
                m13, m23, m33);
    }

    /**
     * Computes estimated true magnetic flux density squared norm using current measured
     * body magnetic flux density and provided parameters when common z-axis is assumed.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the common z-axis case.
     *               Must have length 9.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateCommonAxis(final double[] params) throws EvaluationException {
        final double bx = params[0];
        final double by = params[1];
        final double bz = params[2];

        final double m11 = params[3];

        final double m12 = params[4];
        final double m22 = params[5];

        final double m13 = params[6];
        final double m23 = params[7];
        final double m33 = params[8];

        return evaluate(bx, by, bz, m11, 0.0, 0.0, m12, m22, 0.0,
                m13, m23, m33);
    }

    /**
     * Computes estimated true magnetic flux density squared norm using current measured
     * body magnetic flux density and provided parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param bx  x-coordinate of bias.
     * @param by  y-coordinate of bias.
     * @param bz  z-coordinate of bias.
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
    private double evaluate(final double bx, final double by, final double bz,
                            final double m11, final double m21, final double m31,
                            final double m12, final double m22, final double m32,
                            final double m13, final double m23, final double m33)
            throws EvaluationException {

        // bmeas = M*(btrue + b)

        // btrue = M^-1*bmeas - b

        try {
            if (mBmeas == null) {
                mBmeas = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                        1);
            }
            if (mM == null) {
                mM = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                        BodyMagneticFluxDensity.COMPONENTS);
            }
            if (mInvM == null) {
                mInvM = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                        BodyMagneticFluxDensity.COMPONENTS);
            }
            if (mB == null) {
                mB = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                        1);
            }
            if (mBtrue == null) {
                mBtrue = new Matrix(BodyMagneticFluxDensity.COMPONENTS,
                        1);
            }

            mBmeas.setElementAtIndex(0, mBmeasX);
            mBmeas.setElementAtIndex(1, mBmeasY);
            mBmeas.setElementAtIndex(2, mBmeasZ);

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

            mB.setElementAtIndex(0, bx);
            mB.setElementAtIndex(1, by);
            mB.setElementAtIndex(2, bz);

            mInvM.multiply(mBmeas, mBtrue);
            mBtrue.subtract(mB);

            final double norm = Utils.normF(mBtrue);
            return norm * norm;

        } catch (final AlgebraException e) {
            throw new EvaluationException(e);
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
     * @param unit  unit of magnetic flux density value.
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
