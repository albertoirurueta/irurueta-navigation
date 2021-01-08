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
package com.irurueta.navigation.inertial.calibration.bias;

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.units.Angle;
import com.irurueta.units.Distance;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.io.IOException;
import java.util.Date;
import java.util.GregorianCalendar;

/**
 * Approximately estimated magnetometer biases (hard iron) and noise PSD's by
 * averaging all provided samples when instant, body position and orientation
 * is known while assuming that any soft iron cross coupling errors can be
 * neglected.
 * <p>
 * The estimator must be used when the body where the magnetometer is attached to
 * remains static on the same position with zero velocity and no rotation speed
 * while capturing data.
 * <p>
 * To compute PSD's this estimator assumes that magnetometer samples are obtained at
 * a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, magnetometer sampling rate average can be estimated using
 * {@link com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator}.
 * <p>
 * Notice that in order to compute magnetometer biases (hard iron), instant, body
 * position and orientation must be known to account for expected magnetic field
 * to be sensed.
 * <p>
 * Even though this estimator obtains approximate bias values, the obtained
 * result can be used to initialize some non-linear calibrators to obtain
 * more accurate results, by using bias values as initial hard iron values.
 * Such calibrators are:
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownFrameMagnetometerNonLinearLeastSquaresCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownFrameMagnetometerCalibrator} and any
 * of its subclasses.
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownPositionAndInstantMagnetometerCalibrator}
 * and any of its subclasses.
 * <p>
 * Even though this estimator can compute noise PSD's, if only noise PSD's levels
 * are required, estimators in {@link com.irurueta.navigation.inertial.calibration.noise} package should
 * be used instead.
 * <p>
 * This estimator does NOT compute average bias values over a period of time, it only
 * computes accumulated averages.
 * <p>
 */
public class BodyMagneticFluxDensityBiasEstimator {

    /**
     * Default time interval between accelerometer samples expressed in seconds (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Time interval expressed in seconds (s) between body kinematics samples.
     */
    private double mTimeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Contains body position, velocity (which will always be zero) and orientation
     * resolved around ECEF axes.
     * By default it is assumed that body is located at zero NED coordinates (latitude,
     * longitude and height) and with zero Euler angles representing rotation (roll = 0,
     * pith = 0, yaw = 0), which for Android devices it means that the device is flat
     * on a horizontal surface with the screen facing down.
     */
    private final ECEFFrame mFrame;

    /**
     * Contains year expressed in decimal format.
     */
    private double mYear;

    /**
     * Listener to handle events raised by this estimator.
     */
    private BodyMagneticFluxDensityBiasEstimatorListener mListener;

    /**
     * Contains Earth's magnetic model.
     */
    private WorldMagneticModel mMagneticModel;

    /**
     * World Magnetic Model of Earth.
     */
    private WMMEarthMagneticFluxDensityEstimator mWmmEstimator;

    /**
     * Last provided body magnetic flux density values.
     */
    private BodyMagneticFluxDensity mLastBodyMagneticFluxDensity;

    /**
     * Contains estimated bias of x coordinate of body magnetic flux density
     * expressed in Teslas (T). Notice that bias is equivalent to hard iron
     * component on a magnetometer calibrator.
     */
    private double mBiasX;

    /**
     * Contains estimated bias of y coordinate of body magnetic flux density
     * expressed in Teslas (T). Notice that bias is equivalent to hard iron
     * component on a magnetometer calibrator.
     */
    private double mBiasY;

    /**
     * Contains estimated bias of z coordinate of body magnetic flux density
     * expressed in Teslas (T). Notice that bias is equivalent to hard iron
     * component on a magnetometer calibrator.
     */
    private double mBiasZ;

    /**
     * Contains estimated variance of x coordinate of body magnetic flux density
     * expressed in squared Teslas (T^2).
     */
    private double mVarianceX;

    /**
     * Contains estimated variance of y coordinate of body magnetic flux density
     * expressed in squared Teslas (T^2).
     */
    private double mVarianceY;

    /**
     * Contains estimated variance of z coordinate of body magnetic flux density
     * expressed in squared Teslas (T^2).
     */
    private double mVarianceZ;

    /**
     * Number of processed magnetometer samples.
     */
    private int mNumberOfProcessedSamples;

    /**
     * Number of processed magnetometer samples plus one.
     */
    private int mNumberOfProcessedSamplesPlusOne = 1;

    /**
     * Indicates that estimator is running.
     */
    private boolean mRunning;

    /**
     * Theoretical expected body magnetic flux density for provided instant,
     * body position and orientation, assuming that body remains at the same
     * position (zero velocity).
     * When body remains static, sensed magentic flux density will remain constant
     * for a few minutes respect to provided time instant.
     */
    private BodyMagneticFluxDensity mExpectedBodyMagneticFluxDensity;

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     * This constructor assumes that time is current time instant.
     *
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator()
            throws IOException {
        this(new Date(), (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     * This constructor assumes that time is current time instant.
     *
     * @param nedC coordinate transformation from body to local navigation
     *             (NED) coordinates. This contains orientation respect the horizon
     *             at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, new Date(), (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     * This constructor assumes that time is current time instant.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height)
            throws IOException {
        this(latitude, longitude, height, new Date(), (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     * This constructor assumes that time is current time instant.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height)
            throws IOException {
        this(latitude, longitude, height, new Date(), (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     * This constructor assumes that time is current time instant.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height)
            throws IOException {
        this(latitude, longitude, height, new Date(), (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * This constructor assumes that time is current time instant.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, new Date(), (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * This constructor assumes that time is current time instant.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, new Date(), (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     * This constructor assumes that time is current time instant.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(new Date());
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     * This constructor assumes that time is current time instant.
     *
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, new Date());
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     * This constructor assumes that time is current time instant.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, new Date());
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     * This constructor assumes that time is current time instant.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, new Date());
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     * This constructor assumes that time is current time instant.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, new Date());
        mListener = listener;
    }

    /**
     * Constructor.
     * This constructor assumes that time is current time instant.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, new Date());
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, new Date());
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param year time expressed as decimal year.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(final double year)
            throws IOException {
        this(year, (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC coordinate transformation from body to local navigation
     *             (NED) coordinates. This contains orientation respect the horizon
     *             at current body location.
     * @param year time expressed as decimal year.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC, final double year)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, year, (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param year      time expressed as decimal year.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final double year) throws IOException {
        this(latitude, longitude, height, year, (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param year      time expressed as decimal year.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final double year) throws IOException {
        this(latitude, longitude, height, year, (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param year      time expressed as decimal year.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final double year) throws IOException {
        this(latitude, longitude, height, year, (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param year     time expressed as decimal year.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final double year)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, year, (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param year     time expressed as decimal year.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final double year)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, year, (WorldMagneticModel) null);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param year     time expressed as decimal year.
     * @param listener listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double year,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(year);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param year     time expressed as decimal year.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC,
            final double year,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, year);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param year      time expressed as decimal year.
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final double year,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, year);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param year      time expressed as decimal year.
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final double year,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, year);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param year      time expressed as decimal year.
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final double year,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, year);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param year     time expressed as decimal year.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final double year,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, year);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param year     time expressed as decimal year.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final double year,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, year);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param date a time instance to be converted.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(final Date date) throws IOException {
        this(convertTime(date));
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC coordinate transformation from body to local navigation
     *             (NED) coordinates. This contains orientation respect the horizon
     *             at current body location.
     * @param date a time instance to be converted.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC, final Date date)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, convertTime(date));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param date      a time instance to be converted.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final Date date) throws IOException {
        this(latitude, longitude, height, convertTime(date));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param date      a time instance to be converted.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final Date date) throws IOException {
        this(latitude, longitude, height, convertTime(date));
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param date      a time instance to be converted.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final Date date) throws IOException {
        this(latitude, longitude, height, convertTime(date));
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param date     a time instance to be converted.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final Date date)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, convertTime(date));
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param date     a time instance to be converted.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final Date date)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, convertTime(date));
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param date     a time instance to be converted.
     * @param listener listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Date date,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(convertTime(date), listener);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param date     a time instance to be converted.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC,
            final Date date,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, convertTime(date), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param date      a time instance to be converted.
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final Date date,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param date      a time instance to be converted.
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final Date date,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param date      a time instance to be converted.
     * @param listener  listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final Date date,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), listener);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in NED coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param date     a time instance to be converted.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final Date date,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, convertTime(date), listener);
    }

    /**
     * Constructor.
     *
     * @param position body position expressed in ECEF coordinates.
     * @param nedC     coordinate transformation from body to local navigation
     *                 (NED) coordinates. This contains orientation respect the
     *                 horizon at current body location.
     * @param date     a time instance to be converted.
     * @param listener listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final Date date,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, convertTime(date), listener);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double year, final WorldMagneticModel magneticModel)
            throws IOException {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame());
        mYear = year;
        mMagneticModel = magneticModel;
        initialize();
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the horizon
     *                      at current body location.
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC, final double year,
            final WorldMagneticModel magneticModel)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(nedC));
        mYear = year;
        mMagneticModel = magneticModel;
        initialize();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude expressed in radians (rad).
     * @param longitude     longitude expressed in radians (rad).
     * @param height        height expressed in meters (m).
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final double year, final WorldMagneticModel magneticModel)
            throws IOException {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        mYear = year;
        mMagneticModel = magneticModel;
        initialize();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude.
     * @param longitude     longitude.
     * @param height        height expressed in meters (m).
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final double year, final WorldMagneticModel magneticModel)
            throws IOException {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        mYear = year;
        mMagneticModel = magneticModel;
        initialize();
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude.
     * @param longitude     longitude.
     * @param height        height.
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final double year, final WorldMagneticModel magneticModel)
            throws IOException {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(latitude, longitude, height));
        mYear = year;
        mMagneticModel = magneticModel;
        initialize();
    }

    /**
     * Constructor.
     *
     * @param position      body position expressed in NED coordinates.
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final double year, final WorldMagneticModel magneticModel)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        mFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(
                new NEDFrame(position, nedC));
        mYear = year;
        mMagneticModel = magneticModel;
        initialize();
    }

    /**
     * Constructor.
     *
     * @param position      body position expressed in ECEF coordinates.
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final double year, final WorldMagneticModel magneticModel)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        mFrame = new ECEFFrame(position);
        final NEDFrame nedFrame = ECEFtoNEDFrameConverter
                .convertECEFtoNEDAndReturnNew(mFrame);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mYear = year;
        mMagneticModel = magneticModel;
        initialize();
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param year          time expressed as decimal year.
     * @param listener      listener to handle events raised by this estimator.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double year, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(year, magneticModel);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC,
            final double year, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, year, magneticModel);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude expressed in radians (rad).
     * @param longitude     longitude expressed in radians (rad).
     * @param height        height expressed in meters (m).
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final double year, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, year, magneticModel);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude.
     * @param longitude     longitude.
     * @param height        height expressed in meters (m).
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final double year, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, year, magneticModel);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude.
     * @param longitude     longitude.
     * @param height        height.
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final double year, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, year, magneticModel);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position      body position expressed in NED coordinates.
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final double year,
            final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, year, magneticModel);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position      body position expressed in ECEF coordinates.
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param year          time expressed as decimal year.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final double year, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, year, magneticModel);
        mListener = listener;
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Date date, final WorldMagneticModel magneticModel)
            throws IOException {
        this(convertTime(date), magneticModel);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the horizon
     *                      at current body location.
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC, final Date date,
            final WorldMagneticModel magneticModel)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, convertTime(date), magneticModel);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude expressed in radians (rad).
     * @param longitude     longitude expressed in radians (rad).
     * @param height        height expressed in meters (m).
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final Date date, final WorldMagneticModel magneticModel)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), magneticModel);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude.
     * @param longitude     longitude.
     * @param height        height expressed in meters (m).
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final Date date, final WorldMagneticModel magneticModel)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), magneticModel);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude.
     * @param longitude     longitude.
     * @param height        height.
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final Date date, final WorldMagneticModel magneticModel)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), magneticModel);
    }

    /**
     * Constructor.
     *
     * @param position      body position expressed in NED coordinates.
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final Date date, final WorldMagneticModel magneticModel)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, convertTime(date), magneticModel);
    }

    /**
     * Constructor.
     *
     * @param position      body position expressed in ECEF coordinates.
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final Date date, final WorldMagneticModel magneticModel)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, convertTime(date), magneticModel);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0 and height = 0) and with zero Euler angles representing rotation
     * (roll = 0, pith = 0, yaw = 0), which for Android devices it means that the
     * device is flat on a horizontal surface with the screen facing down.
     *
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Date date, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(convertTime(date), magneticModel, listener);
    }

    /**
     * Constructor.
     * It is assumed that body is located at zero NED coordinates (latitude = 0,
     * longitude = 0, and height = 0) with provided orientation.
     *
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final CoordinateTransformation nedC,
            final Date date, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(nedC, convertTime(date), magneticModel, listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude expressed in radians (rad).
     * @param longitude     longitude expressed in radians (rad).
     * @param height        height expressed in meters (m).
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final double latitude, final double longitude, final double height,
            final Date date, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), magneticModel,
                listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude.
     * @param longitude     longitude.
     * @param height        height expressed in meters (m).
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final double height,
            final Date date, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), magneticModel,
                listener);
    }

    /**
     * Constructor.
     * It is assumed that body has zero Euler angles representing rotation (roll = 0,
     * pitch = 0, yaw = 0) respect the horizon at provided body location.
     * For Android devices this means that the device is flat on a horizontal surface
     * with the screen facing down.
     *
     * @param latitude      latitude.
     * @param longitude     longitude.
     * @param height        height.
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws IOException if initialization of world magnetic model fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final Angle latitude, final Angle longitude, final Distance height,
            final Date date, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws IOException {
        this(latitude, longitude, height, convertTime(date), magneticModel,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position      body position expressed in NED coordinates.
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final NEDPosition position, final CoordinateTransformation nedC,
            final Date date, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, convertTime(date), magneticModel, listener);
    }

    /**
     * Constructor.
     *
     * @param position      body position expressed in ECEF coordinates.
     * @param nedC          coordinate transformation from body to local navigation
     *                      (NED) coordinates. This contains orientation respect the
     *                      horizon at current body location.
     * @param date          a time instance to be converted.
     * @param magneticModel world magnetic model of Earth or null if default
     *                      model is used.
     * @param listener      listener to handle events raised by this estimator.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not
     *                                                       from body to local
     *                                                       navigation coordinates.
     * @throws IOException                                   if initialization of
     *                                                       world magnetic model
     *                                                       fails.
     */
    public BodyMagneticFluxDensityBiasEstimator(
            final ECEFPosition position, final CoordinateTransformation nedC,
            final Date date, final WorldMagneticModel magneticModel,
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException, IOException {
        this(position, nedC, convertTime(date), magneticModel, listener);
    }


    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples expressed in seconds (s).
     *
     * @return time interval between body kinematics samples.
     */
    public double getTimeInterval() {
        return mTimeInterval;
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples expressed in seconds (s).
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        mTimeInterval = timeInterval;
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @return time interval between body kinematics samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(mTimeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @param result instance where time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(mTimeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration + gyroscope)
     * samples.
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(convertTime(timeInterval));
    }

    /**
     * Gets current body position expressed in ECEF coordinates.
     *
     * @return current body position expressed in ECEF coordinates.
     */
    public ECEFPosition getEcefPosition() {
        return mFrame.getECEFPosition();
    }

    /**
     * Gets current body position expressed in ECEF coordinates.
     *
     * @param result instance where current body position will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        mFrame.getECEFPosition(result);
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param position current body position to be set.
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final ECEFPosition position) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(position);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param x x position resolved around ECEF axes and expressed in meters (m).
     * @param y y position resolved around ECEF axes and expressed in meters (m).
     * @param z z position resolved around ECEF axes and expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final double x, final double y, final double z)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setCoordinates(x, y, z);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param x x position resolved around ECEF axes.
     * @param y y position resolved around ECEF axes.
     * @param z z position resolved around ECEF axes.
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(
            final Distance x, final Distance y, final Distance z)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPositionCoordinates(x, y, z);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets current body position expressed in ECEF coordinates.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setEcefPosition(final Point3D position) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(position);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Gets ECEF frame containing current body position and orientation expressed
     * in ECEF coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @return ECEF frame containing current body position and orientation resolved
     * around ECEF axes.
     */
    public ECEFFrame getEcefFrame() {
        return new ECEFFrame(mFrame);
    }

    /**
     * Gets ECEF frame containing current body position and orientation expressed
     * in ECEF coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @param result instance where ECEF frame containing current body position and
     *               orientation resolved around ECEF axes will be stored.
     */
    public void getEcefFrame(final ECEFFrame result) {
        mFrame.copyTo(result);
    }

    /**
     * Gets NED frame containing current body position and orientation expressed
     * in NED coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @return NED frame containing current body position and orientation resolved
     * around NED axes.
     */
    public NEDFrame getNedFrame() {
        return ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(mFrame);
    }

    /**
     * Gets NED frame containing current body position and orientation expressed
     * in NED coordinates. Frame also contains body velocity, but it is always
     * assumed to be zero during calibration.
     *
     * @param result instance where NED frame containing current body position and
     *               orientation resolved around NED axes will be stored.
     */
    public void getNedFrame(final NEDFrame result) {
        ECEFtoNEDFrameConverter.convertECEFtoNED(mFrame, result);
    }

    /**
     * Gets current body position expressed in NED coordinates.
     *
     * @return current body position expressed in NED coordinates.
     */
    public NEDPosition getNedPosition() {
        return getNedFrame().getPosition();
    }

    /**
     * Gets current body position expressed in NED coordinates.
     *
     * @param result instance where current body position will be stored.
     */
    public void getNedPosition(final NEDPosition result) {
        getNedFrame().getPosition(result);
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param position current body position to be set.
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(final NEDPosition position)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(position);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate expressed in radians (rad).
     * @param longitude longitude NED coordinate expressed in radians (rad).
     * @param height    height NED coordinate expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final double latitude, final double longitude, final double height)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate.
     * @param longitude longitude NED coordinate.
     * @param height    height NED coordinate expressed in meters (m).
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final Angle latitude, final Angle longitude, final double height)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets current body position expressed in NED coordinates.
     *
     * @param latitude  latitude NED coordinate.
     * @param longitude longitude NED coordinate.
     * @param height    height NED coordinate.
     * @throws LockedException if estimator is currently running.
     */
    public void setNedPosition(
            final Angle latitude, final Angle longitude, final Distance height)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Gets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that returned orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizon at current body position.
     * Typically it is more convenient to use {@link #getNedC()} to obtain orientation
     * relative to the ground or horizon at current body position. For instance, on
     * Android devices a NED orientation with Euler angles (roll = 0, pitch = 0,
     * yaw = 0) means that the device is laying flat on a horizontal surface with the
     * screen facing down towards the ground.
     *
     * @return current body orientation resolved on ECEF axes.
     */
    public CoordinateTransformation getEcefC() {
        return mFrame.getCoordinateTransformation();
    }

    /**
     * Gets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that returned orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizon at current body position.
     * Typically it is more convenient to use {@link #getNedC()} to obtain orientation
     * relative to the ground or horizon at current body position. For instance, on
     * Android devices a NED orientation with Euler angles (roll = 0, pitch = 0,
     * yaw = 0) means that the device is laying flat on a horizontal surface with the
     * screen facing down towards the ground.
     *
     * @param result instance where current body orientation resolved on ECEF axes
     *               will be stored.
     */
    public void getEcefC(final CoordinateTransformation result) {
        mFrame.getCoordinateTransformation(result);
    }

    /**
     * Sets current body orientation as a transformation from body to ECEF coordinates.
     * Notice that ECEF orientation refers to ECEF Earth axes, which means that
     * orientation is not relative to the ground or horizont at current body position.
     * Typically it is more convenient to use
     * {@link #setNedC(CoordinateTransformation)} to specify orientation relative to
     * the ground or horizon at current body position.
     * For instance, on Android devices a NED orientation with Euler angles (roll = 0,
     * pitch = 0, yaw = 0) means that the device is laying flat on a horizontal surface
     * with the screen facing down towards the ground.
     *
     * @param ecefC body orientation resolved on ECEF axes to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     */
    public void setEcefC(final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Gets current body orientation as a transformation from body to NED coordinates.
     * Notice that returned orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @return current body orientation resolved on NED axes.
     */
    public CoordinateTransformation getNedC() {
        return getNedFrame().getCoordinateTransformation();
    }

    /**
     * Gets current body orientation as a transformation from body to NED coordinates.
     * Notice that returned orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @param result instance where current body orientation resolved on NED axes
     *               will be stored.
     */
    public void getNedC(final CoordinateTransformation result) {
        getNedFrame().getCoordinateTransformation(result);
    }

    /**
     * Sets current body orientation as a transformation from body to NED coordinates.
     * Notice that provided orientation refers to current local position. This means
     * that two equal NED orientations will transform into different ECEF orientations
     * if the body is located at different positions.
     * As a reference, on Android devices a NED orientation with Euler angles
     * (roll = 0, pitch = 0, yaw = 0) means that the device is laying flat on a
     * horizontal surface with the screen facing down towards the ground.
     *
     * @param nedC orientation resolved on NED axes to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     */
    public void setNedC(final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param nedPosition position expressed on NED coordinates.
     * @param nedC        body to NED coordinate transformation indicating
     *                    body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(NEDPosition)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final NEDPosition nedPosition, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(nedPosition);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(double, double, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final double latitude, final double longitude, final double height,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final Angle latitude, final Angle longitude, final double height,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position and orientation both expressed on NED coordinates.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param nedC      body to NED coordinate transformation indicating
     *                  body orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, Distance)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setNedPositionAndNedOrientation(
            final Angle latitude, final Angle longitude, final Distance height,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param ecefC        body to ECEF coordinate transformation indicating body
     *                     orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(ECEFPosition)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final ECEFPosition ecefPosition, final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(ecefPosition);
        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param x     x coordinate of ECEF position expressed in meters (m).
     * @param y     y coordinate of ECEF position expressed in meters (m).
     * @param z     z coordinate of ECEF position expressed in meters (m).
     * @param ecefC body to ECEF coordinate transformation indicating body
     *              orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(double, double, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final double x, final double y, final double z,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setCoordinates(x, y, z);
        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param x     x coordinate of ECEF position.
     * @param y     y coordinate of ECEF position.
     * @param z     z coordinate of ECEF position.
     * @param ecefC body to ECEF coordinate transformation indicating body
     *              orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Distance, Distance, Distance)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final Distance x, final Distance y, final Distance z,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPositionCoordinates(x, y, z);
        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position and orientation both expressed on ECEF coordinates.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @param ecefC    body to ECEF coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Point3D)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setEcefPositionAndEcefOrientation(
            final Point3D position, final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(position);
        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param position position expressed on NED coordinates.
     * @param ecefC    body to ECEF coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(NEDPosition)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final NEDPosition position,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(position);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude expressed in radians (rad).
     * @param longitude longitude expressed in radians (rad).
     * @param height    height expressed in meters (m).
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(double, double, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final double latitude, final double longitude, final double height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height expressed in meters (m).
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, double)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final Angle latitude, final Angle longitude, final double height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position expressed on NED coordinates and orientation respect to ECEF
     * axes.
     *
     * @param latitude  latitude.
     * @param longitude longitude.
     * @param height    height.
     * @param ecefC     body to ECEF coordinate transformation indicating body
     *                  orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to ECEF coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setNedPosition(Angle, Angle, Distance)
     * @see #setEcefC(CoordinateTransformation)
     */
    public void setNedPositionAndEcefOrientation(
            final Angle latitude, final Angle longitude, final Distance height,
            final CoordinateTransformation ecefC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setPosition(latitude, longitude, height);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);
        mFrame.setCoordinateTransformation(ecefC);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param ecefPosition position expressed on ECEF coordinates.
     * @param nedC         body to NED coordinate transformation indicating body
     *                     orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(ECEFPosition)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final ECEFPosition ecefPosition, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(ecefPosition);

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param x    x coordinate of ECEF position expressed in meters (m).
     * @param y    y coordinate of ECEF position expressed in meters (m).
     * @param z    z coordinate of ECEF position expressed in meters (m).
     * @param nedC body to NED coordinate transformation indicating body
     *             orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(double, double, double)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final double x, final double y, final double z,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setCoordinates(x, y, z);

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param x    x coordinate of ECEF position.
     * @param y    y coordinate of ECEF position.
     * @param z    z coordinate of ECEF position.
     * @param nedC body to NED coordinate transformation indicating body
     *             orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Distance, Distance, Distance)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final Distance x, final Distance y, final Distance z,
            final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPositionCoordinates(x, y, z);

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets position expressed on ECEF coordinates and orientation respect to
     * NED axes.
     * In order to preserve provided orientation, first position is set and
     * then orientation is applied.
     *
     * @param position position resolved around ECEF axes and expressed in meters (m).
     * @param nedC     body to NED coordinate transformation indicating body
     *                 orientation.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not from
     *                                                       body to NED coordinates.
     * @throws LockedException                               if estimator is currently
     *                                                       running.
     * @see #setEcefPosition(Point3D)
     * @see #setNedC(CoordinateTransformation)
     */
    public void setEcefPositionAndNedOrientation(
            final Point3D position, final CoordinateTransformation nedC)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mFrame.setPosition(position);

        final NEDFrame nedFrame = getNedFrame();
        nedFrame.setCoordinateTransformation(nedC);
        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, mFrame);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Gets year expressed in decimal format.
     *
     * @return year expressed in decimal format.
     */
    public double getYear() {
        return mYear;
    }

    /**
     * Sets year expressed in decimal format.
     *
     * @param year year expressed in decimal format.
     * @throws LockedException if estimator is running.
     */
    public void setYear(final double year) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mYear = year;

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets decimal year from provided date instance.
     *
     * @param date a date instance containing a timestamp.
     * @throws LockedException if estimator is running.
     */
    public void setTime(final Date date) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mYear = convertTime(date);

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Sets decimal year from provided calendar instance.
     *
     * @param calendar a calendar instance containing a timestamp.
     * @throws LockedException if estimator is running.
     */
    public void setTime(final GregorianCalendar calendar)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mYear = convertTime(calendar);

        rebuildExpectedBodyMagneticFluxDensity();
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
     * If not provided a default model will be loaded internally.
     *
     * @param magneticModel Earth's magnetic model to be set.
     * @throws LockedException if calibrator is currently running.
     * @throws IOException     if initialization of world magnetic model fails.
     */
    public void setMagneticModel(final WorldMagneticModel magneticModel)
            throws LockedException, IOException {
        if (mRunning) {
            throw new LockedException();
        }
        mMagneticModel = magneticModel;
        initialize();
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public BodyMagneticFluxDensityBiasEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(
            final BodyMagneticFluxDensityBiasEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets last provided body magnetic flux density values or null if not
     * available.
     *
     * @return last provided body magnetic flux density values or null.
     */
    public BodyMagneticFluxDensity getLastBodyMagneticFluxDensity() {
        return mLastBodyMagneticFluxDensity != null ?
                new BodyMagneticFluxDensity(mLastBodyMagneticFluxDensity) : null;
    }

    /**
     * Gets last provided body magnetic flux density values.
     *
     * @param result instance where last provided body magnetic flux density will
     *               be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getLastBodyMagneticFluxDensity(final BodyMagneticFluxDensity result) {
        if (mLastBodyMagneticFluxDensity != null) {
            mLastBodyMagneticFluxDensity.copyTo(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated bias of x coordinate of body magnetic flux density
     * expressed in Teslas (T). Notice that bias is equivalent to hard iron
     * component on a magnetometer calibrator.
     *
     * @return bias of x coordinate of body magnetic flux density.
     */
    public double getBiasX() {
        return mBiasX;
    }

    /**
     * Gets estimated bias of x coordinate of body magnetic flux density.
     * Notice that bias is equivalent to hard iron component on a magnetometer
     * calibrator.
     *
     * @return bias of x coordinate of body magnetic flux density.
     */
    public MagneticFluxDensity getBiasXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mBiasX, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated bias of x coordinate of body magnetic flux density.
     * Notice that bias is equivalent to hard iron component on a magnetometer
     * calibrator.
     *
     * @param result instance where bias of x coordinate of body magnetic flux
     *               density will be stored.
     */
    public void getBiasXAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mBiasX);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated bias of y coordinate of body magnetic flux density
     * expressed in Teslas (T). Notice that bias is equivalent to hard iron
     * component on a magnetometer calibrator.
     *
     * @return bias of y coordinate of body magnetic flux density.
     */
    public double getBiasY() {
        return mBiasY;
    }

    /**
     * Gets estimated bias of y coordinate of body magnetic flux density.
     * Notice that bias is equivalent to hard iron component on a
     * magnetometer calibrator.
     *
     * @return bias of y coordinate of body magnetic flux density.
     */
    public MagneticFluxDensity getBiasYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mBiasY, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated bias of y coordinate of body magnetic flux density.
     * Notice that bias is equivalent to hard iron component on a
     * magnetometer calibrator.
     *
     * @param result instance where bias of y coordinate of body magnetic flux
     *               density will be stored.
     */
    public void getBiasYAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mBiasY);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated bias of z coordinate of body magnetic flux density
     * expressed in Teslas (T). Notice that bias is equivalent to hard iron
     * component on a magnetometer calibrator.
     *
     * @return bias of z coordinate of body magnetic flux density.
     */
    public double getBiasZ() {
        return mBiasZ;
    }

    /**
     * Gets estimated bias of z coordinate of body magnetic flux density.
     * Notice that bias is equivalent to hard iron component on a magnetometer
     * calibrator.
     *
     * @return bias of z coordinate of body magnetic flux density.
     */
    public MagneticFluxDensity getBiasZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mBiasZ, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated bias of z coordinate of body magnetic flux density.
     * Notice that bias is equivalent to hard iron component on a magnetometer
     * calibrator.
     *
     * @param result instance where bias of z coordinate of body magnetic flux
     *               density will be stored.
     */
    public void getBiasZAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mBiasZ);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated bias of body magnetic flux density.
     *
     * @return estimated bias of magnetic flux density.
     */
    public MagneticFluxDensityTriad getBiasTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                mBiasX, mBiasY, mBiasZ);
    }

    /**
     * Gets estimated bias of body magnetic flux density.
     *
     * @param result instance where bias of body magnetic flux density will
     *               be stored.
     */
    public void getBiasTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(mBiasX, mBiasY, mBiasZ,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated variance of x coordinate of body magnetic flux density
     * expressed in squared Teslas (T^2).
     *
     * @return estimated variance of x coordinate of body magnetic flux density.
     */
    public double getVarianceX() {
        return mVarianceX;
    }

    /**
     * Gets estimated variance of y coordinate of body magnetic flux density
     * expressed in squared Teslas (T^2).
     *
     * @return estimated variance of y coordinate of body magnetic flux density.
     */
    public double getVarianceY() {
        return mVarianceY;
    }

    /**
     * Gets estimated variance of z coordinate of body magnetic flux density
     * expressed in squared Teslas (T^2).
     *
     * @return estimated variance of z coordinate of body magnetic flux density.
     */
    public double getVarianceZ() {
        return mVarianceZ;
    }

    /**
     * Gets estimated standard deviation of x coordinate of body magnetic flux
     * density expressed in Teslas (T).
     *
     * @return estimated standard deviation of x coordinate of body magnetic
     * flux density.
     */
    public double getStandardDeviationX() {
        return Math.sqrt(mVarianceX);
    }

    /**
     * Gets estimated standard deviation of x coordinate of body magnetic flux
     * density.
     *
     * @return estimated standard deviation of x coordinate of body magnetic
     * flux density.
     */
    public MagneticFluxDensity getStandardDeviationXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getStandardDeviationX(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated standard deviation of x coordinate of body magnetic flux
     * density.
     *
     * @param result instance where estimated standard deviation of x coordinate
     *               of body magnetic flux density will be stored.
     */
    public void getStandardDeviationXAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(getStandardDeviationX());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated standard deviation of y coordinate of body magnetic flux
     * density expressed in Teslas (T).
     *
     * @return estimated standard deviation of y coordinate of body magnetic
     * flux density.
     */
    public double getStandardDeviationY() {
        return Math.sqrt(mVarianceY);
    }

    /**
     * Gets estimated standard deviation of y coordinate of body magnetic flux
     * density.
     *
     * @return estimated standard deviation of y coordinate of body magnetic
     * flux density.
     */
    public MagneticFluxDensity getStandardDeviationYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getStandardDeviationY(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated standard deviation of y coordinate of body magnetic flux
     * density.
     *
     * @param result instance where estimated standard deviation of y coordinate
     *               of body magnetic flux density will be stored.
     */
    public void getStandardDeviationYAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(getStandardDeviationY());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated standard deviation of z coordinate of body magnetic flux
     * density expressed in Teslas (T).
     *
     * @return estimated standard deviation of z coordinate of body magnetic
     * flux density.
     */
    public double getStandardDeviationZ() {
        return Math.sqrt(mVarianceZ);
    }

    /**
     * Gets estimated standard deviation of z coordinate of body magnetic flux
     * density.
     *
     * @return estimated standard deviation of z coordinate of body magnetic
     * flux density.
     */
    public MagneticFluxDensity getStandardDeviationZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getStandardDeviationZ(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated standard deviation of z coordinate of body magnetic flux
     * density.
     *
     * @param result instance where estimated standard deviation of z coordinate
     *               of body magnetic flux density will be stored.
     */
    public void getStandardDeviationZAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(getStandardDeviationZ());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets estimated standard deviation of body magnetic flux density.
     *
     * @return estimated standard deviation of body magnetic flux density.
     */
    public MagneticFluxDensityTriad getStandardDeviationTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                getStandardDeviationX(),
                getStandardDeviationY(),
                getStandardDeviationZ());
    }

    /**
     * Gets estimated standard deviation of body magnetic flux density.
     *
     * @param result instance where estimated standard deviation of body magnetic
     *               flux density will be stored.
     */
    public void getStandardDeviationTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(getStandardDeviationX(),
                getStandardDeviationY(), getStandardDeviationZ(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets average of estimated standard deviation of body magnetic flux density
     * expressed in Teslas (T).
     *
     * @return average of estimated standard deviation of body magnetic flux
     * density.
     */
    public double getAverageStandardDeviation() {
        return (getStandardDeviationX() + getStandardDeviationY()
                + getStandardDeviationZ()) / 3.0;
    }

    /**
     * Gets average of estimated standard deviation of body magnetic flux density.
     *
     * @return average of estimated standard deviation of body magnetic flux
     * density.
     */
    public MagneticFluxDensity getAverageStandardDeviationAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getAverageStandardDeviation(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets average of estimated standard deviation of body magnetic flux density.
     *
     * @param result instance where average of estimated standard deviation of
     *               body magnetic flux density will be stored.
     */
    public void getAverageStandardDeviationAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        result.setValue(getAverageStandardDeviation());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets magnetometer noise PSD (Power Spectral Density) on x axis expressed
     * in (T^2 * s).
     *
     * @return magnetometer noise PSD on x axis.
     */
    public double getPsdX() {
        return mVarianceX * mTimeInterval;
    }

    /**
     * Gets magnetometer noise PSD (Power Spectral Density) on y axis expressed
     * in (T^2 * s).
     *
     * @return magnetometer noise PSD on y axis.
     */
    public double getPsdY() {
        return mVarianceY * mTimeInterval;
    }

    /**
     * Gets magnetometer noise PSD (Power Spectral Density) on z axis expressed
     * in (T^2 * s).
     *
     * @return magnetometer noise PSD on z axis.
     */
    public double getPsdZ() {
        return mVarianceZ * mTimeInterval;
    }

    /**
     * Gets magnetometer noise root PSD (Power Spectral Density) on x axis
     * expressed in (T * s^0.5).
     *
     * @return magnetometer noise root PSD on x axis.
     */
    public double getRootPsdX() {
        return Math.sqrt(getPsdX());
    }

    /**
     * Gets magnetometer noise root PSD (Power Spectral Density) on y axis
     * expressed in (T * s^0.5).
     *
     * @return magnetometer noise root PSD on y axis.
     */
    public double getRootPsdY() {
        return Math.sqrt(getPsdY());
    }

    /**
     * Gets magnetometer noise root PSD (Power Spectral Density) on z axis
     * expressed in (T * s^0.5).
     *
     * @return magnetometer noise root PSD on z axis.
     */
    public double getRootPsdZ() {
        return Math.sqrt(getPsdZ());
    }

    /**
     * Gets average magnetometer noise PSD (Power Spectral Density) among
     * x,y,z components expressed as (T^2 * s).
     *
     * @return average magnetometer noise PSD.
     */
    public double getAvgPsd() {
        return (getPsdX() + getPsdY() + getPsdZ()) / 3.0;
    }

    /**
     * Gets magnetometer root noise root PSD (Power Spectral Density) which is
     * the norm of root PSD components expressed as (T * s^0.5).
     *
     * @return average magnetometer noise root PSD.
     */
    public double getRootPsd() {
        return Math.sqrt(getPsdX() + getPsdY() + getPsdZ());
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getNumberOfProcessedSamples() {
        return mNumberOfProcessedSamples;
    }

    /**
     * Indicates whether estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Gets theoretically expected body magnetic flux density for provided instant,
     * body position and orientation, assuming that body remains at the same
     * position (zero velocity).
     * When body remains static, sensed magentic flux density will remain constant
     * for a few minutes respect to provided time instant.
     *
     * @return expected body magnetic flux density.
     */
    public BodyMagneticFluxDensity getExpectedBodyMagneticFluxDensity() {
        return new BodyMagneticFluxDensity(mExpectedBodyMagneticFluxDensity);
    }

    /**
     * Gets theoretically expected body magnetic flux density for provided instant,
     * body position and orientation, assuming that body remains at the same
     * position (zero velocity).
     * When body remains static, sensed magentic flux density will remain constant
     * for a few minutes respect to provided time instant.
     *
     * @param result instance where expected body magnetic flux density will be
     *               stored.
     */
    public void getExpectedBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity result) {
        mExpectedBodyMagneticFluxDensity.copyTo(result);
    }

    /**
     * Adds a sample of body magnetic flux density. If estimator is already
     *
     * @param bodyMagneticFluxDensity body magnetic flux density to be added
     *                                and processed.
     * @throws LockedException if estimator is currently running.
     */
    public void addBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity bodyMagneticFluxDensity)
            throws LockedException {

        if (mRunning) {
            throw new LockedException();
        }

        mRunning = true;

        if (mLastBodyMagneticFluxDensity == null && mListener != null) {
            mListener.onStart(this);
        }

        final double bx = bodyMagneticFluxDensity.getBx();
        final double by = bodyMagneticFluxDensity.getBy();
        final double bz = bodyMagneticFluxDensity.getBz();

        final double expectedBx = mExpectedBodyMagneticFluxDensity.getBx();
        final double expectedBy = mExpectedBodyMagneticFluxDensity.getBy();
        final double expectedBz = mExpectedBodyMagneticFluxDensity.getBz();

        final double diffBx = bx - expectedBx;
        final double diffBy = by - expectedBy;
        final double diffBz = bz - expectedBz;

        // compute biases
        final double tmp = (double) mNumberOfProcessedSamples
                / (double) mNumberOfProcessedSamplesPlusOne;
        mBiasX = mBiasX * tmp + diffBx / (double) mNumberOfProcessedSamplesPlusOne;
        mBiasY = mBiasY * tmp + diffBy / (double) mNumberOfProcessedSamplesPlusOne;
        mBiasZ = mBiasZ * tmp + diffBz / (double) mNumberOfProcessedSamplesPlusOne;

        // compute variances
        final double diffBiasX = diffBx - mBiasX;
        final double diffBiasY = diffBy - mBiasY;
        final double diffBiasZ = diffBz - mBiasZ;

        final double diffBiasX2 = diffBiasX * diffBiasX;
        final double diffBiasY2 = diffBiasY * diffBiasY;
        final double diffBiasZ2 = diffBiasZ * diffBiasZ;

        mVarianceX = mVarianceX * tmp
                + diffBiasX2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceY = mVarianceY * tmp
                + diffBiasY2 / (double) mNumberOfProcessedSamplesPlusOne;
        mVarianceZ = mVarianceZ * tmp
                + diffBiasZ2 / (double) mNumberOfProcessedSamplesPlusOne;

        mLastBodyMagneticFluxDensity = bodyMagneticFluxDensity;

        mNumberOfProcessedSamples++;
        mNumberOfProcessedSamplesPlusOne++;

        if (mListener != null) {
            mListener.onBodyMagneticFluxDensityAdded(this);
        }

        mRunning = false;
    }

    /**
     * Resets current estimator.
     *
     * @return true if estimator was successfully reset, false if no reset
     * was needed.
     * @throws LockedException if estimator is currently running.
     */
    public boolean reset() throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (mNumberOfProcessedSamples == 0) {
            return false;
        }

        mRunning = true;
        mLastBodyMagneticFluxDensity = null;
        mBiasX = 0.0;
        mBiasY = 0.0;
        mBiasZ = 0.0;
        mVarianceX = 0.0;
        mVarianceY = 0.0;
        mVarianceZ = 0.0;
        mNumberOfProcessedSamples = 0;
        mNumberOfProcessedSamplesPlusOne = 1;

        if (mListener != null) {
            mListener.onReset(this);
        }

        mRunning = false;

        return true;
    }

    /**
     * Converts a time instant contained ina date object to a
     * decimal year.
     *
     * @param date a time instance to be converted.
     * @return converted value expressed in decimal years.
     */
    public static double convertTime(final Date date) {
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
    public static double convertTime(final GregorianCalendar calendar) {
        return WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
    }

    /**
     * Converts provided time instance to seconds.
     *
     * @param time instance to be converted.
     * @return obtained conversion in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Initializes world magnetic model and estimates expected body magnetic flux
     * density.
     *
     * @throws IOException if world magnetic model loading fails.
     */
    private void initialize() throws IOException {
        if (mMagneticModel != null) {
            mWmmEstimator = new WMMEarthMagneticFluxDensityEstimator(mMagneticModel);
        } else {
            mWmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }

        rebuildExpectedBodyMagneticFluxDensity();
    }

    /**
     * Rebuilds expected body magnetic flux density based on current instant,
     * location and body orientation.
     */
    private void rebuildExpectedBodyMagneticFluxDensity() {
        final NEDFrame nedFrame = ECEFtoNEDFrameConverter
                .convertECEFtoNEDAndReturnNew(mFrame);

        final double latitude = nedFrame.getLatitude();
        final double longitude = nedFrame.getLongitude();
        final double height = nedFrame.getHeight();

        final CoordinateTransformation cbn = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final CoordinateTransformation cnb = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
        nedFrame.getCoordinateTransformation(cbn);
        cbn.inverse(cnb);

        final NEDMagneticFluxDensity earthB = mWmmEstimator.estimate(
                latitude, longitude, height, mYear);

        // estimate expected body mangetic flux density taking into
        // account body attitude (inverse of frame orientation) and
        // estimated Earth magnetic flux density
        if (mExpectedBodyMagneticFluxDensity == null) {
            mExpectedBodyMagneticFluxDensity = new BodyMagneticFluxDensity();
        }
        BodyMagneticFluxDensityEstimator.estimate(earthB, cnb,
                mExpectedBodyMagneticFluxDensity);
    }
}
