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
package com.irurueta.navigation.geodesic.wmm;

import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

/**
 * Estimates Earth magnetic flux density  resolved around NED frame at
 * a given Earth location.
 */
public class WMMEarthMagneticFluxDensityEstimator {

    /**
     * Guaranteed accuracy of estimated angles by the WMM during
     * the valid timespan of a model.
     * Accuracy is expressed in degrees and refers to estimated
     * declination and dip angles.
     */
    public static final double ANGLE_ACCURACY_DEGREES = 5e-3;

    /**
     * Guaranteed accuracy of estimated angles by the WMM during
     * the valid timespan of a model.
     * Accuracy is expressed in radians and refers to estimated
     * declination and dip angles.
     */
    public static final double ANGLE_ACCURACY_RADIANS =
            Math.toRadians(ANGLE_ACCURACY_DEGREES);

    /**
     * Guaranteed accuracy of estimated magnetic flux density
     * values by the WMM during the valid timespan of a model.
     * This value refers to intensity norm, vertical intensity,
     * horizontal intensity, north intensity and east intensity.
     */
    public static final double INTENSITY_ACCURACY = 5e-2;

    /**
     * Number of coefficients.
     */
    private static final int N = WorldMagneticModel.N;

    /**
     * Coefficients file.
     */
    private static final String COEFFICIENTS_FILE = "wmm.cof";

    /**
     * Mean radius of IAU-66 ellipsoid expressed in Km.
     */
    private static final double RE_KM = 6371.2;

    /**
     * Converts to nanos.
     */
    private static final double FROM_NANO = 1e-9;

    /**
     * Time value used in previous calculation.
     * This is used to save on calculation time if some
     * inputs don't change.
     * Old time is expressed in decimal years.
     */
    private Double mOldTime;

    /**
     * Geodetic height (a.k.a. altitude) value used in previous
     * calculation.
     * This is used to save on calculation time if some
     * inputs don't change.
     * Old height is expressed in Kilometers (Km).
     */
    private Double mOldHeight;

    /**
     * Old geodetic latitude value used in previous
     * calculation.
     * This is used to save on calculation time if some
     * inputs don't change.
     * Old latitude is expressed in degrees (deg).
     */
    private Double mOldLatitude;

    /**
     * Old geodetic longitude value used in previous
     * calculation.
     * This is used to save on calculation time if some
     * inputs don't change.
     * Old longitude is expressed in degrees (deg).
     */
    private Double mOldLongitude;

    /**
     * Geomagnetic declination in degrees.
     * East is positive, West is negative.
     * (The negative of variation).
     */
    private double mDec;

    /**
     * Geomagnetic inclination in degrees.
     * Down is positive, up is negative.
     */
    private double mDip;

    /**
     * Geomagnetic total intensity expressed in nano Teslas (nT)
     */
    private double mTi;

    /**
     * A World Magnetic Model containing all required coefficients.
     */
    private final WorldMagneticModel mModel;

    /**
     * The time adjusted geomagnetic gauss coefficients (nt).
     */
    private final double[][] mTc = new double[N][N];

    /**
     * The theta derivative of p(n,m) (unnormalized).
     */
    private final double[][] mDp = new double[N][N];

    /**
     * The sine of (m*spherical coord. longitude).
     */
    private final double[] mSp = new double[N];

    /**
     * The cosine of (m*spherical coord. longitude).
     */
    private final double[] mCp = new double[N];

    /**
     * The associated Legendre polynomials for m=1 (unnormalized).
     */
    private final double[] mPp = new double[N];

    /**
     * The north-south field intensity expressed in nano Teslas (nT).
     */
    private double mBx;

    /**
     * The east-west field intensity expressed in nano Teslas (nT).
     */
    private double mBy;

    /**
     * The vertical field intensity positive downward expressed
     * in nano Teslas (nT)
     */
    private double mBz;

    /**
     * The horizontal field intensity expressed in nano Teslas (nT)
     */
    private double mBh;

    /**
     * Semi-major axis of WGS-84 ellipsoid, in Km, squared.
     */
    private final double mA2;

    /**
     * Semi-minor axis of WGS-84 ellipsoid, in Km, squared.
     */
    private final double mB2;

    /**
     * The difference between the squared semi axes.
     * c2 = a2 - b2
     */
    private final double mC2;

    /**
     * {@link #mA2} squared.
     */
    private final double mA4;

    /**
     * The difference between a4 and b4
     * c4 = a4 - b4
     */
    private final double mC4;

    // Below: internal values being reused. These values are only
    // recalculated if the height (altitude) changes.

    private double mR;

    private double mCa;

    private double mSa;

    private double mCt;

    private double mSt;

    /**
     * Constructor.
     *
     * @throws IOException if an I/O error occurs while loading
     *                     model coefficients.
     */
    public WMMEarthMagneticFluxDensityEstimator()
            throws IOException {
        this(WMMLoader.loadFromResource(COEFFICIENTS_FILE));
    }

    /**
     * Constructor.
     *
     * @param model a World Magnetic Model.
     * @throws NullPointerException if provided model is null.
     */
    public WMMEarthMagneticFluxDensityEstimator(
            final WorldMagneticModel model) {
        if (model == null) {
            throw new NullPointerException();
        }
        mModel = model;
        mSp[0] = 0.0;
        mCp[0] = 1.0;
        mPp[0] = 1.0;

        // semi-major axis of WGS-84 ellipsoid, in Km (6378.137 Km).
        final double a = DistanceConverter.convert(
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                DistanceUnit.METER, DistanceUnit.KILOMETER);

        // semi-minor axis of WGS-84 ellipsoid, in Km (6356.7523142 Km).
        final double b = DistanceConverter.convert(
                Constants.EARTH_POLAR_RADIUS_WGS84,
                DistanceUnit.METER, DistanceUnit.KILOMETER);
        mA2 = a * a;
        mB2 = b * b;
        mC2 = mA2 - mB2;
        mA4 = mA2 * mA2;

        final double b4 = mB2 * mB2;
        mC4 = mA4 - b4;
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final double latitude, final double longitude) {
        final double defaultTime = mModel.epoch
                + WorldMagneticModel.LIFESPAN / 2.0;
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                0.0, defaultTime);
        return Math.toRadians(mDec);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final Angle latitude, final Angle longitude) {
        return getDeclination(convertAngle(latitude),
                convertAngle(longitude));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where magnetic field declination will be
     *                  stored.
     */
    public void getDeclinationAsAngle(
            final double latitude, final double longitude,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(latitude, longitude));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final double latitude, final double longitude) {
        return new Angle(getDeclination(latitude, longitude),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param result    instance where magnetic field declination will be
     *                  stored.
     */
    public void getDeclinationAsAngle(
            final Angle latitude, final Angle longitude,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(latitude, longitude));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final Angle latitude, final Angle longitude) {
        return new Angle(getDeclination(latitude, longitude),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final double latitude, final double longitude,
            final double height, final double year) {
        final double heightKm = DistanceConverter.convert(height,
                DistanceUnit.METER,
                DistanceUnit.KILOMETER);
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                heightKm, year);
        return Math.toRadians(mDec);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return getDeclination(latitude, longitude, height,
                convertTime(calendar));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final double latitude, final double longitude,
            final double height, final Date time) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(time);
        return getDeclination(latitude, longitude, height, calendar);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return getDeclination(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                year);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return getDeclination(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                calendar);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return getDeclination(convertAngle(latitude), convertAngle(longitude),
                convertDistance(height), time);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final NEDPosition position, final double year) {
        return getDeclination(position.getLatitude(), position.getLongitude(),
                position.getHeight(), year);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final NEDPosition position, final GregorianCalendar calendar) {
        return getDeclination(position.getLatitude(), position.getLongitude(),
                position.getHeight(), calendar);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return magnetic field declination expressed in radians.
     */
    public double getDeclination(
            final NEDPosition position, final Date time) {
        return getDeclination(position.getLatitude(),
                position.getLongitude(), position.getHeight(), time);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @param result    instance where magnetic field declination will be
     *                  stored.
     */
    public void getDeclinationAsAngle(
            final double latitude, final double longitude,
            final double height, final double year, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(latitude, longitude, height, year));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final double latitude, final double longitude,
            final double height, final double year) {
        return new Angle(getDeclination(latitude, longitude, height, year),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @param result    instance where magnetic field declination will be
     *                  stored.
     */
    public void getDeclinationAsAngle(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(latitude, longitude, height,
                calendar));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return new Angle(getDeclination(latitude, longitude, height,
                calendar), AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @param result    instance where magnetic field declination will be
     *                  stored.
     */
    public void getDeclinationAsAngle(
            final double latitude, final double longitude,
            final double height, final Date time, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(latitude, longitude, height, time));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final double latitude, final double longitude,
            final double height, final Date time) {
        return new Angle(getDeclination(latitude, longitude, height, time),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @param result    instance where magnetic field declination will be
     *                  stored.
     */
    public void getDeclinationAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(latitude, longitude, height, year));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return new Angle(getDeclination(latitude, longitude, height, year),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @param result    instance where magnetic field declination will be
     *                  stored.
     */
    public void getDeclinationAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(latitude, longitude, height,
                calendar));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return new Angle(getDeclination(latitude, longitude, height,
                calendar), AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @param result    instance where magnetic field declination will be
     *                  stored.
     */
    public void getDeclinationAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(latitude, longitude, height, time));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * <p>
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return new Angle(getDeclination(latitude, longitude, height, time),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @param result   instance where magnetic field declination will be
     *                 stored.
     */
    public void getDeclinationAsAngle(
            final NEDPosition position, final double year,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(position, year));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final NEDPosition position, final double year) {
        return new Angle(getDeclination(position, year), AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @param result   instance where magnetic field declination will be
     *                 stored.
     */
    public void getDeclinationAsAngle(
            final NEDPosition position, final GregorianCalendar calendar,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(position, calendar));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final NEDPosition position, final GregorianCalendar calendar) {
        return new Angle(getDeclination(position, calendar),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @param result   instance where magnetic field declination will be
     *                 stored.
     */
    public void getDeclinationAsAngle(
            final NEDPosition position, final Date time,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDeclination(position, time));
    }

    /**
     * Returns the declination from the Department of Defense geomagnetic
     * model and data, in radians.
     * The magnetic heading + declination is the true heading of a device
     * in terms of geographical north pole.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return magnetic field declination.
     */
    public Angle getDeclinationAsAngle(
            final NEDPosition position, final Date time) {
        return new Angle(getDeclination(position, time),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(final double latitude, final double longitude) {
        final double defaultTime = mModel.epoch
                + WorldMagneticModel.LIFESPAN / 2.0;
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                0.0, defaultTime);
        return Math.toRadians(mDip);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(final Angle latitude, final Angle longitude) {
        return getDip(convertAngle(latitude), convertAngle(longitude));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where magnetic field dip angle will be
     *                  stored.
     */
    public void getDipAsAngle(
            final double latitude, final double longitude,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(latitude, longitude));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final double latitude, final double longitude) {
        return new Angle(getDip(latitude, longitude), AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param result    instance where magnetic field dip angle will be
     *                  stored.
     */
    public void getDipAsAngle(
            final Angle latitude, final Angle longitude,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(latitude, longitude));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final Angle latitude, final Angle longitude) {
        return new Angle(getDip(latitude, longitude), AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(
            final double latitude, final double longitude,
            final double height, final double year) {
        final double heightKm = DistanceConverter.convert(height,
                DistanceUnit.METER,
                DistanceUnit.KILOMETER);
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                heightKm, year);
        return Math.toRadians(mDip);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return getDip(latitude, longitude, height,
                convertTime(calendar));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(
            final double latitude, final double longitude,
            final double height, final Date time) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(time);
        return getDip(latitude, longitude, height, calendar);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return getDip(convertAngle(latitude), convertAngle(longitude),
                convertDistance(height), year);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return getDip(convertAngle(latitude), convertAngle(longitude),
                convertDistance(height), calendar);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return getDip(convertAngle(latitude), convertAngle(longitude),
                convertDistance(height), time);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(final NEDPosition position, final double year) {
        return getDip(position.getLatitude(), position.getLongitude(),
                position.getHeight(), year);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(
            final NEDPosition position,
            final GregorianCalendar calendar) {

        return getDip(position.getLatitude(), position.getLongitude(),
                position.getHeight(), calendar);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return the magnetic field dip angle expressed in radians.
     */
    public double getDip(
            final NEDPosition position, final Date time) {
        return getDip(position.getLatitude(), position.getLongitude(),
                position.getHeight(), time);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @param result    instance where magnetic field dip angle will be
     *                  stored.
     */
    public void getDipAsAngle(
            final double latitude, final double longitude,
            final double height, final double year, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(latitude, longitude, height, year));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final double latitude, final double longitude,
            final double height, final double year) {
        return new Angle(getDip(latitude, longitude, height, year),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @param result    instance where magnetic field dip angle will be
     *                  stored.
     */
    public void getDipAsAngle(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(latitude, longitude, height, calendar));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return new Angle(getDip(latitude, longitude, height, calendar),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @param result    instance where magnetic field dip angle will be
     *                  stored.
     */
    public void getDipAsAngle(
            final double latitude, final double longitude,
            final double height, final Date time, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(latitude, longitude, height, time));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final double latitude, final double longitude,
            final double height, final Date time) {
        return new Angle(getDip(latitude, longitude, height, time),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @param result    instance where magnetic field dip angle will be
     *                  stored.
     */
    public void getDipAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(latitude, longitude, height, year));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return new Angle(getDip(latitude, longitude, height, year),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @param result    instance where magnetic field dip angle will be
     *                  stored.
     */
    public void getDipAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(latitude, longitude, height, calendar));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return new Angle(getDip(latitude, longitude, height, calendar),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @param result    instance where magnetic field dip angle will be
     *                  stored.
     */
    public void getDipAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(latitude, longitude, height, time));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return new Angle(getDip(latitude, longitude, height, time),
                AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @param result   instance where magnetic field dip angle will be
     *                 stored.
     */
    public void getDipAsAngle(
            final NEDPosition position, final double year,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(position, year));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final NEDPosition position, final double year) {
        return new Angle(getDip(position, year), AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @param result   instance where magnetic field dip angle will be
     *                 stored.
     */
    public void getDipAsAngle(
            final NEDPosition position, final GregorianCalendar calendar,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(position, calendar));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final NEDPosition position,
            final GregorianCalendar calendar) {
        return new Angle(getDip(position, calendar), AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @param result   instance where magnetic field dip angle will be
     *                 stored.
     */
    public void getDipAsAngle(
            final NEDPosition position, final Date time,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getDip(position, time));
    }

    /**
     * Returns the magnetic field dip angle from the Department of
     * Defense geomagnetic model and data, in radians.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return the magnetic field dip angle.
     */
    public Angle getDipAsAngle(
            final NEDPosition position, final Date time) {
        return new Angle(getDip(position, time), AngleUnit.RADIANS);
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final double latitude, final double longitude) {
        final double defaultTime = mModel.epoch
                + WorldMagneticModel.LIFESPAN / 2.0;
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                0.0, defaultTime);
        return mTi * FROM_NANO;
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final Angle latitude, final Angle longitude) {
        return getIntensity(convertAngle(latitude),
                convertAngle(longitude));
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final double latitude, final double longitude,
            final double height, final double year) {
        final double heightKm = DistanceConverter.convert(height,
                DistanceUnit.METER,
                DistanceUnit.KILOMETER);
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                heightKm, year);
        return mTi * FROM_NANO;
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return getIntensity(latitude, longitude, height,
                convertTime(calendar));
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final double latitude, final double longitude,
            final double height, final Date time) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(time);
        return getIntensity(latitude, longitude, height, calendar);
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return getIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                year);
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return getIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                calendar);
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return getIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                time);
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final NEDPosition position, final double year) {
        return getIntensity(position.getLatitude(), position.getLongitude(),
                position.getHeight(), year);
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final NEDPosition position, final GregorianCalendar calendar) {
        return getIntensity(position.getLatitude(), position.getLongitude(),
                position.getHeight(), calendar);
    }

    /**
     * Returns the magnetic field intensity from the Department of
     * Defense geomagnetic model and data expressed in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return magnetic field strength expressed in Teslas (T).
     */
    public double getIntensity(
            final NEDPosition position, final Date time) {
        return getIntensity(position.getLatitude(), position.getLongitude(),
                position.getHeight(), time);
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final double latitude, final double longitude) {
        final double defaultTime = mModel.epoch
                + WorldMagneticModel.LIFESPAN / 2.0;
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                0.0, defaultTime);
        return mBh * FROM_NANO;
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final Angle latitude, final Angle longitude) {
        return getHorizontalIntensity(convertAngle(latitude),
                convertAngle(longitude));
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final double latitude, final double longitude,
            final double height, final double year) {
        final double heightKm = DistanceConverter.convert(height,
                DistanceUnit.METER,
                DistanceUnit.KILOMETER);
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                heightKm, year);
        return mBh * FROM_NANO;
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return getHorizontalIntensity(latitude, longitude, height,
                convertTime(calendar));
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final double latitude, final double longitude,
            final double height, final Date time) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(time);
        return getHorizontalIntensity(latitude, longitude, height,
                calendar);
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return getHorizontalIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                year);
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return getHorizontalIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                calendar);
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return getHorizontalIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                time);
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final NEDPosition position, final double year) {
        return getHorizontalIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), year);
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final NEDPosition position, final GregorianCalendar calendar) {
        return getHorizontalIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), calendar);
    }

    /**
     * Returns the horizontal magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return the horizontal magnetic field strength expressed in
     * Teslas (T).
     */
    public double getHorizontalIntensity(
            final NEDPosition position, final Date time) {
        return getHorizontalIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), time);
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final double latitude, final double longitude) {
        final double defaultTime = mModel.epoch
                + WorldMagneticModel.LIFESPAN / 2.0;
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                0.0, defaultTime);
        return mBz * FROM_NANO;
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final Angle latitude, final Angle longitude) {
        return getVerticalIntensity(convertAngle(latitude),
                convertAngle(longitude));
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final double latitude, final double longitude,
            final double height, final double year) {
        final double heightKm = DistanceConverter.convert(height,
                DistanceUnit.METER,
                DistanceUnit.KILOMETER);
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                heightKm, year);
        return mBz * FROM_NANO;
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return getVerticalIntensity(latitude, longitude, height,
                convertTime(calendar));
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final double latitude, final double longitude,
            final double height, final Date time) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(time);
        return getVerticalIntensity(latitude, longitude, height,
                calendar);
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return getVerticalIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                year);
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return getVerticalIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                calendar);
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return getVerticalIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                time);
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final NEDPosition position, final double year) {
        return getVerticalIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), year);
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final NEDPosition position, final GregorianCalendar calendar) {
        return getVerticalIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), calendar);
    }

    /**
     * Returns the vertical magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return the vertical magnetic field strength expressed in
     * Teslas (T).
     */
    public double getVerticalIntensity(
            final NEDPosition position, final Date time) {
        return getVerticalIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), time);
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final double latitude, final double longitude) {
        final double defaultTime = mModel.epoch
                + WorldMagneticModel.LIFESPAN / 2.0;
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                0.0, defaultTime);
        return mBx * FROM_NANO;
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final Angle latitude, final Angle longitude) {
        return getNorthIntensity(convertAngle(latitude),
                convertAngle(longitude));
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final double latitude, final double longitude,
            final double height, final double year) {
        final double heightKm = DistanceConverter.convert(height,
                DistanceUnit.METER,
                DistanceUnit.KILOMETER);
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                heightKm, year);
        return mBx * FROM_NANO;
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return getNorthIntensity(latitude, longitude, height,
                convertTime(calendar));
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final double latitude, final double longitude,
            final double height, final Date time) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(time);
        return getNorthIntensity(latitude, longitude, height,
                calendar);
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return getNorthIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                year);
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return getNorthIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                calendar);
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return getNorthIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                time);
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final NEDPosition position, final double year) {
        return getNorthIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), year);
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final NEDPosition position, final GregorianCalendar calendar) {
        return getNorthIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), calendar);
    }

    /**
     * Returns the northerly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return the northerly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getNorthIntensity(
            final NEDPosition position, final Date time) {
        return getNorthIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), time);
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final double latitude, final double longitude) {
        final double defaultTime = mModel.epoch
                + WorldMagneticModel.LIFESPAN / 2.0;
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                0.0, defaultTime);
        return mBy * FROM_NANO;
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final Angle latitude, final Angle longitude) {
        return getEastIntensity(convertAngle(latitude),
                convertAngle(longitude));
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final double latitude, final double longitude,
            final double height, final double year) {
        final double heightKm = DistanceConverter.convert(height,
                DistanceUnit.METER,
                DistanceUnit.KILOMETER);
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                heightKm, year);
        return mBy * FROM_NANO;
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        return getEastIntensity(latitude, longitude, height,
                convertTime(calendar));
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final double latitude, final double longitude,
            final double height, final Date time) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(time);
        return getEastIntensity(latitude, longitude, height, calendar);
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        return getEastIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                year);
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        return getEastIntensity(convertAngle(latitude),
                convertAngle(longitude),
                convertDistance(height), calendar);
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        return getEastIntensity(convertAngle(latitude),
                convertAngle(longitude), convertDistance(height),
                time);
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final NEDPosition position, final double year) {
        return getEastIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), year);
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final NEDPosition position, final GregorianCalendar calendar) {
        return getEastIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), calendar);
    }

    /**
     * Returns the easterly magnetic field intensity from the
     * Department of Defense geomagnetic model and data expressed
     * in nano Teslas.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return the easterly component of the magnetic field strength
     * expressed in Teslas (T).
     */
    public double getEastIntensity(
            final NEDPosition position, final Date time) {
        return getEastIntensity(position.getLatitude(),
                position.getLongitude(), position.getHeight(), time);
    }

    /**
     * Estimates Earth magnetic flux density.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param result    instance where magnetic flux will be stored resolved
     *                  around NED frame.
     */
    public void estimate(
            final double latitude, final double longitude,
            final NEDMagneticFluxDensity result) {
        final double defaultTime = mModel.epoch
                + WorldMagneticModel.LIFESPAN / 2.0;
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                0.0, defaultTime);
        final double bn = mBx * FROM_NANO;
        final double be = mBy * FROM_NANO;
        final double bd = mBz * FROM_NANO;

        result.setCoordinates(bn, be, bd);
    }

    /**
     * Estimates Earth magnetic flux density.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final double latitude, final double longitude) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(latitude, longitude, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param result    instance where magnetic flux will be stored resolved
     *                  around NED frame.
     */
    public void estimate(
            final Angle latitude, final Angle longitude,
            final NEDMagneticFluxDensity result) {
        estimate(convertAngle(latitude), convertAngle(longitude),
                result);
    }

    /**
     * Estimates Earth magnetic flux density.
     * This method uses default altitude (0.0 - mean sea level) and time
     * (half way through the valid 5 year period of the model).
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final Angle latitude, final Angle longitude) {
        return estimate(convertAngle(latitude), convertAngle(longitude));
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @param result    instance where magnetic flux will be stored resolved
     *                  around NED frame.
     */
    public void estimate(
            final double latitude, final double longitude,
            final double height, final double year,
            final NEDMagneticFluxDensity result) {

        final double heightKm = DistanceConverter.convert(height,
                DistanceUnit.METER,
                DistanceUnit.KILOMETER);
        calcGeoMag(Math.toDegrees(latitude), Math.toDegrees(longitude),
                heightKm, year);
        final double bn = mBx * FROM_NANO;
        final double be = mBy * FROM_NANO;
        final double bd = mBz * FROM_NANO;

        result.setCoordinates(bn, be, bd);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param year      year expressed in decimal years.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final double latitude, final double longitude,
            final double height, final double year) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(latitude, longitude, height, year, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @param result    instance where magnetic flux will be stored resolved
     *                  around NED frame.
     */
    public void estimate(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar,
            final NEDMagneticFluxDensity result) {
        estimate(latitude, longitude, height, convertTime(calendar),
                result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param calendar  a calendar containing a specific instant.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final double latitude, final double longitude,
            final double height, final GregorianCalendar calendar) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(latitude, longitude, height, calendar, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @param result    instance where magnetic flux will be stored resolved
     *                  around NED frame.
     */
    public void estimate(
            final double latitude, final double longitude,
            final double height, final Date time,
            final NEDMagneticFluxDensity result) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(time);
        estimate(latitude, longitude, height, calendar, result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param time      a specific time instant.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final double latitude, final double longitude,
            final double height, final Date time) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(latitude, longitude, height, time, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @param result    instance where magnetic flux will be stored resolved
     *                  around NED frame.
     */
    public void estimate(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year,
            final NEDMagneticFluxDensity result) {
        estimate(convertAngle(latitude), convertAngle(longitude),
                convertDistance(height), year, result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param year      year expressed in decimal years.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final Angle latitude, final Angle longitude,
            final Distance height, final double year) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(latitude, longitude, height, year, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @param result    instance where magnetic flux will be stored resolved
     *                  around NED frame.
     */
    public void estimate(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar,
            final NEDMagneticFluxDensity result) {
        estimate(convertAngle(latitude), convertAngle(longitude),
                convertDistance(height), calendar, result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param calendar  a calendar containing a specific instant.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final Angle latitude, final Angle longitude,
            final Distance height, final GregorianCalendar calendar) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(latitude, longitude, height, calendar, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @param result    instance where magnetic flux will be stored resolved
     *                  around NED frame.
     */
    public void estimate(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time,
            final NEDMagneticFluxDensity result) {
        estimate(convertAngle(latitude), convertAngle(longitude),
                convertDistance(height), time, result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param latitude  latitude angle.
     * @param longitude longitude angle.
     * @param height    height.
     * @param time      a specific time instant.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final Angle latitude, final Angle longitude,
            final Distance height, final Date time) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(latitude, longitude, height, time, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @param result   instance where magnetic flux will be stored resolved
     *                 around NED frame.
     */
    public void estimate(
            final NEDPosition position, final double year,
            final NEDMagneticFluxDensity result) {
        estimate(position.getLatitude(), position.getLongitude(),
                position.getHeight(), year, result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param year     year expressed in decimal years.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final NEDPosition position, final double year) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(position, year, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @param result   instance where magnetic flux will be stored resolved
     *                 around NED frame.
     */
    public void estimate(
            final NEDPosition position, final GregorianCalendar calendar,
            final NEDMagneticFluxDensity result) {
        estimate(position.getLatitude(), position.getLongitude(),
                position.getHeight(), calendar, result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param calendar a calendar containing a specific instant.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final NEDPosition position, final GregorianCalendar calendar) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(position, calendar, result);
        return result;
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @param result   instance where magnetic flux will be stored resolved
     *                 around NED frame.
     */
    public void estimate(
            final NEDPosition position, final Date time,
            final NEDMagneticFluxDensity result) {
        estimate(position.getLatitude(), position.getLongitude(),
                position.getHeight(), time, result);
    }

    /**
     * Estimates Earth magnetic flux density.
     *
     * @param position a position expressed in geodetic coordinates.
     * @param time     a specific time instant.
     * @return Earth magnetic flux density resolved around NED frame.
     */
    public NEDMagneticFluxDensity estimate(
            final NEDPosition position, final Date time) {
        final NEDMagneticFluxDensity result = new NEDMagneticFluxDensity();
        estimate(position, time, result);
        return result;
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
        final int year = calendar.get(Calendar.YEAR);
        final double daysInYear;
        if (calendar.isLeapYear(year)) {
            daysInYear = 366.0;
        } else {
            daysInYear = 365.0;
        }

        return year + calendar.get(Calendar.DAY_OF_YEAR) / daysInYear;
    }

    /**
     * Computes the declination (dec), inclination (dip), total intensity
     * (TI) and grid variation (GV - polar regions only, referenced to grid
     * north of polar stereographic projection) of the Earth's magnetic field
     * in geodetic coordinates from the coefficients of the current official
     * department of defense (DoD) spherical harmonic World Magnetic Model
     * (WMM-2010).  The WMM series of models is updated every 5 years on
     * January 1st of those years which are divisible by 5 (i.e. 1980, 1985,
     * 1990, etc.) by the Navan Oceanographic Office in cooperation with the
     * British Geological Survey (BGS). The model is based on geomagnetic
     * survey measurements from aircraft, satellite and geomagnetic
     * observatories.
     *
     * Accuracy:
     * In ocean areas at the Earth's surface over the entire 5 year life of
     * a degree and order 12 spherical harmonic model such as WMM-95, the
     * estimated RMS errors for the various magnetic components are:
     *     DEC  - 0.5 degrees
     *     DIP  - 0.5 degrees
     *     TI   - 200.0 nano Teslas (nT)
     *     GV   - 0.5 Degrees
     *
     * Other magnetic components that can be derived from these four by
     * simple trigonometric relations will have the following approximate
     * errors over ocean areas:
     *     X    - 140 nT (North)
     *     Y    - 140 nT (East)
     *     Z    - 200 nT (Vertical) Positive is down
     *     H    - 200 nT (Horizontal)
     *
     * Over land the RMS errors are expected to be somewhat higher, although
     * the RMS errors for DEC, DIP and GV are still estimated to be less than
     * 0.5 degree, for the entire 5-year life of the model at the Earth's
     * surface. The other component errors over land are more difficult to
     * estimate and so are not given.
     *
     * The accuracy at any given time of all four geomagnetic parameters
     * depends on the geomagnetic latitude. The errors are least at the
     * equator and greatest at the magnetic poles.
     *
     * It is very important to note that a degree and order 12 model, such
     * as WMM-2010 describes only the long wavelength spatial magnetic
     * fluctuations due to Earth's core. Not included in the WMM series
     * models are intermediate and short wavelength spatial fluctuations of
     * the geomagnetic field which originate in the Earth's mantle and crust.
     * Consequently, isolated angular errors at various positions on the
     * surface (primarily over land, in continental margins and over oceanic
     * seamounts, ridges and trenches) of several degrees may be expected.
     * Also not included in the model are nonsecular temporal fluctuations
     * of the geomagnetic field of magnetospheric and ionospheric origin.
     * During magnetic storms, temporal fluctuations can cause substantial
     * deviations of the geomagnetic field from model values. In arctic and
     * antarctic regions, as well as in equatorial regions, deviations from
     * model values are both frequent and persistent.
     *
     * If the required declination accuracy is more stringent than the WMM
     * series of models provide, then the user is advised to request special
     * (regional or local) surveys be performed and models prepared by the
     * USGS, which operates the US geomagnetic observatories.
     *
     * @param latitude  the latitude in decimal degrees.
     * @param longitude the longitude in decimal degrees.
     * @param height    the height (altitude) in kilometers.
     * @param year      the date as a decimal year.
     */
    private void calcGeoMag(
            final double latitude,
            final double longitude,
            final double height,
            final double year) {

        final double dt = year - mModel.epoch;
        final double rlon = Math.toRadians(longitude);
        final double rlat = Math.toRadians(latitude);
        final double srlon = Math.sin(rlon);
        final double srlat = Math.sin(rlat);
        final double crlon = Math.cos(rlon);
        final double crlat = Math.cos(rlat);
        final double srlat2 = srlat * srlat;
        final double crlat2 = crlat * crlat;
        mSp[1] = srlon;
        mCp[1] = crlon;

        // Convert from geodetic coords to spherical coords.
        if (mOldHeight == null || height != mOldHeight
                || mOldLatitude == null || latitude != mOldLatitude) {
            final double q = Math.sqrt(mA2 - mC2 * srlat2);
            final double q1 = height * q;
            final double q2 = ((q1 + mA2) / (q1 + mB2)) * ((q1 + mA2) / (q1 + mB2));
            mCt = srlat / Math.sqrt(q2 * crlat2 + srlat2);
            mSt = Math.sqrt(1.0 - (mCt * mCt));
            final double r2 = ((height * height) + 2.0 * q1 + (mA4 - mC4 * srlat2) / (q * q));
            mR = Math.sqrt(r2);
            final double mD = Math.sqrt(mA2 * crlat2 + mB2 * srlat2);
            mCa = (height + mD) / mR;
            mSa = mC2 * crlat * srlat / (mR * mD);
        }
        if (mOldLongitude == null || longitude != mOldLongitude) {
            for (int m = 2; m <= WorldMagneticModel.MAX_ORDER; m++) {
                mSp[m] = mSp[1] * mCp[m - 1] + mCp[1] * mSp[m - 1];
                mCp[m] = mCp[1] * mCp[m - 1] - mSp[1] * mSp[m - 1];
            }
        }

        final double aor = RE_KM / mR;
        double ar = aor * aor;
        double br = 0, bt = 0, bp = 0, bpp = 0;

        for (int n = 1; n <= WorldMagneticModel.MAX_ORDER; n++) {
            ar = ar * aor;
            for (int m = 0, D3 = 1, D4 = (n + m + D3) / D3; D4 > 0; D4--, m += D3) {

                // compute unnormalized associated Legendre polynomials
                // and derivatives via recursion relations
                if (mOldHeight == null || height != mOldHeight
                        || mOldLatitude == null || latitude != mOldLatitude) {
                    if (n == m) {
                        mModel.snorm[n + m * N] = mSt * mModel.snorm[n - 1 + (m - 1) * N];
                        mDp[m][n] = mSt * mDp[m - 1][n - 1] + mCt * mModel.snorm[n - 1 + (m - 1) * N];
                    }
                    if (n == 1 && m == 0) {
                        mModel.snorm[n + m * N] = mCt * mModel.snorm[n - 1 + m * N];
                        mDp[m][n] = mCt * mDp[m][n - 1] - mSt * mModel.snorm[n - 1 + m * N];
                    }
                    if (n > 1 && n != m) {
                        if (m > n - 2) {
                            mModel.snorm[n - 2 + m * N] = 0.0;
                        }
                        if (m > n - 2) {
                            mDp[m][n - 2] = 0.0;
                        }
                        mModel.snorm[n + m * N] = mCt * mModel.snorm[n - 1 + m * N]
                                - mModel.k[m][n] * mModel.snorm[n - 2 + m * N];
                        mDp[m][n] = mCt * mDp[m][n - 1] - mSt * mModel.snorm[n - 1 + m * N]
                                - mModel.k[m][n] * mDp[m][n - 2];
                    }
                }

                // time adjust the Gauss coefficients

                if (mOldTime == null || year != mOldTime) {
                    mTc[m][n] = mModel.c[m][n] + dt * mModel.cd[m][n];

                    if (m != 0) {
                        mTc[n][m - 1] = mModel.c[n][m - 1] + dt * mModel.cd[n][m - 1];
                    }
                }

                // accumulate terms of the spherical harmonic expansions
                double temp1, temp2;
                double par = ar * mModel.snorm[n + m * N];
                if (m == 0) {
                    temp1 = mTc[m][n] * mCp[m];
                    temp2 = mTc[m][n] * mSp[m];
                } else {
                    temp1 = mTc[m][n] * mCp[m] + mTc[n][m - 1] * mSp[m];
                    temp2 = mTc[m][n] * mSp[m] - mTc[n][m - 1] * mCp[m];
                }

                bt = bt - ar * temp1 * mDp[m][n];
                bp += (mModel.fm[m] * temp2 * par);
                br += (mModel.fn[n] * temp1 * par);

                // Special case: North/sourth geographic poles

                if (mSt == 0.0 && m == 1) {
                    if (n == 1)
                        mPp[n] = mPp[n - 1];
                    else
                        mPp[n] = mCt * mPp[n - 1] - mModel.k[m][n] * mPp[n - 2];
                    double parp = ar * mPp[n];
                    bpp += (mModel.fm[m] * temp2 * parp);
                }

            }    //for(m...)

        }    //for(n...)


        if (mSt == 0.0) {
            bp = bpp;
        } else {
            bp /= mSt;
        }

        // Rotate magnetic vector components from spherical to
        // geodetic coordinates.
        // by is the east-west field component
        // bx is the north-south field component
        // bz is the vertical field component.
        mBx = -bt * mCa - br * mSa;
        mBy = bp;
        mBz = bt * mSa - br * mCa;

        // Compute declination (DEC), INCLINATION (DIP) and
        // total intensity (TI)

        mBh = Math.sqrt((mBx * mBx) + (mBy * mBy));
        mTi = Math.sqrt((mBh * mBh) + (mBz * mBz));
        //	Calculate the declination.
        mDec = Math.toDegrees(Math.atan2(mBy, mBx));
        mDip = Math.toDegrees(Math.atan2(mBz, mBh));

        mOldTime = year;
        mOldHeight = height;
        mOldLatitude = latitude;
        mOldLongitude = longitude;
    }

    /**
     * Converts provided angle instance to radians.
     *
     * @param angle angle to be converted.
     * @return converted value expressed in radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Converts provided distance to meters.
     *
     * @param distance distance to be converted.
     * @return converted value expressed in meters.
     */
    private static double convertDistance(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(),
                distance.getUnit(), DistanceUnit.METER);
    }
}
