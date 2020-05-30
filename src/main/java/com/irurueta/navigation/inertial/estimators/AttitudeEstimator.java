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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.geodesic.wmm.EarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

import java.io.IOException;
import java.util.Date;
import java.util.GregorianCalendar;

/**
 * Estimates the attitude of a body by taking into account both accelerometer
 * and magnetometer measurements.
 * Although {@link LevelingEstimator} and {@link LevelingEstimator2} can
 * estimate body attitude just by using IMU measurements (accelerometer +
 * gyroscope), they require the gyroscope to be accurate enough (aviation
 * grade accuracy) to obtain reliable results.
 * When the gyroscope is known to not be accurate (e.g. mobile devices),
 * this class should be used instead to use a magnetometer to obtain a
 * reliable attitude estimation.
 */
public class AttitudeEstimator {

    /**
     * Estimator of the World Magnetic Model.
     * This is used to determine magnetic declination angle at a given Earth
     * position and instant.
     */
    private final WMMEarthMagneticFluxDensityEstimator mWMMEstimator;

    /**
     * Constructor.
     * This constructor loads the default WMM, which is valid from
     * 2020, January 1st until 2025, January 1st.
     *
     * @throws IOException if an I/O error occurs while loading the WMM
     *                     model.
     */
    public AttitudeEstimator() throws IOException {
        mWMMEstimator = new WMMEarthMagneticFluxDensityEstimator();
    }

    /**
     * Constructor.
     *
     * @param model a World Magnetic Model. The model is only valid for
     *              a certain timespan.
     * @throws NullPointerException if provided model is null.
     */
    public AttitudeEstimator(final WorldMagneticModel model) {
        mWMMEstimator = new WMMEarthMagneticFluxDensityEstimator(model);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param latitude  Earth position latitude expressed in radians (rad).
     * @param longitude Earth position longitude expressed in radians (rad).
     * @param height    Earth position height respect average sea level expressed
     *                  in meters (m).
     * @param year      a year expressed in decimal value.
     * @param fx        x-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fy        y-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fz        z-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param bx        x coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param by        y coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param bz        z coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param result    instance where body attitude will be stored.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public void getAttitude(
            final double latitude, final double longitude, final double height,
            final double year, final double fx, final double fy, final double fz,
            final double bx, final double by, final double bz,
            final CoordinateTransformation result) {
        final double declination = mWMMEstimator.getDeclination(
                latitude, longitude, height, year);
        getAttitude(latitude, height, fx, fy, fz, bx, by, bz, declination,
                result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param latitude  Earth position latitude expressed in radians (rad).
     * @param longitude Earth position longitude expressed in radians (rad).
     * @param height    Earth position height respect average sea level expressed
     *                  in meters (m).
     * @param year      a year expressed in decimal value.
     * @param fx        x-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fy        y-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fz        z-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param bx        x coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param by        y coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param bz        z coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @return a coordinate transformation containing body attitude.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public CoordinateTransformation getAttitude(
            final double latitude, final double longitude, final double height,
            final double year, final double fx, final double fy, final double fz,
            final double bx, final double by, final double bz) {
        final double declination = mWMMEstimator.getDeclination(
                latitude, longitude, height, year);
        return getAttitude(latitude, height, fx, fy, fz, bx, by, bz,
                declination);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param latitude  Earth position latitude expressed in radians (rad).
     * @param longitude Earth position longitude expressed in radians (rad).
     * @param height    Earth position height respect average sea level expressed
     *                  in meters (m).
     * @param calendar  a calendar indicating a given timestamp.
     * @param fx        x-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fy        y-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fz        z-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param bx        x coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param by        y coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param bz        z coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param result    instance where body attitude will be stored.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public void getAttitude(
            final double latitude, final double longitude, final double height,
            final GregorianCalendar calendar, final double fx, final double fy,
            final double fz, final double bx, final double by, final double bz,
            final CoordinateTransformation result) {
        final double declination = mWMMEstimator.getDeclination(
                latitude, longitude, height, calendar);
        getAttitude(latitude, height, fx, fy, fz, bx, by, bz, declination,
                result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param latitude  Earth position latitude expressed in radians (rad).
     * @param longitude Earth position longitude expressed in radians (rad).
     * @param height    Earth position height respect average sea level expressed
     *                  in meters (m).
     * @param calendar  a calendar indicating a given timestamp.
     * @param fx        x-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fy        y-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fz        z-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param bx        x coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param by        y coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param bz        z coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @return a coordinate transformation containing body attitude.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public CoordinateTransformation getAttitude(
            final double latitude, final double longitude, final double height,
            final GregorianCalendar calendar, final double fx, final double fy,
            final double fz, final double bx, final double by, final double bz) {
        final double declination = mWMMEstimator.getDeclination(
                latitude, longitude, height, calendar);
        return getAttitude(latitude, height, fx, fy, fz, bx, by, bz,
                declination);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param latitude  Earth position latitude expressed in radians (rad).
     * @param longitude Earth position longitude expressed in radians (rad).
     * @param height    Earth position height respect average sea level expressed
     *                  in meters (m).
     * @param timestamp a timestamp.
     * @param fx        x-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fy        y-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fz        z-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param bx        x coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param by        y coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param bz        z coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param result    instance where body attitude will be stored.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public void getAttitude(
            final double latitude, final double longitude, final double height,
            final Date timestamp, final double fx, final double fy,
            final double fz, final double bx, final double by, final double bz,
            final CoordinateTransformation result) {
        final double declination = mWMMEstimator.getDeclination(
                latitude, longitude, height, timestamp);
        getAttitude(latitude, height, fx, fy, fz, bx, by, bz, declination,
                result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param latitude  Earth position latitude expressed in radians (rad).
     * @param longitude Earth position longitude expressed in radians (rad).
     * @param height    Earth position height respect average sea level expressed
     *                  in meters (m).
     * @param timestamp a timestamp.
     * @param fx        x-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fy        y-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param fz        z-coordinate of measured body specific force
     *                  expressed in meters per squared second (m/s^2).
     * @param bx        x coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param by        y coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @param bz        z coordinate of measured magnetic flux density resolved
     *                  around body axes and expressed in Teslas (T).
     * @return a coordinate transformation containing body attitude.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public CoordinateTransformation getAttitude(
            final double latitude, final double longitude, final double height,
            final Date timestamp, final double fx, final double fy,
            final double fz, final double bx, final double by, final double bz) {
        final double declination = mWMMEstimator.getDeclination(
                latitude, longitude, height, timestamp);
        return getAttitude(latitude, height, fx, fy, fz, bx, by, bz,
                declination);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param position   Earth position resolved in NED coordinates.
     * @param year       a year expressed in decimal value.
     * @param kinematics body kinematics containing measured specific force.
     * @param b          measured body magnetic flux density.
     * @param result     instance where body attitude will be stored.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public void getAttitude(
            final NEDPosition position,
            final double year,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final CoordinateTransformation result) {
        getAttitude(position.getLatitude(), position.getLongitude(),
                position.getHeight(), year, kinematics.getFx(),
                kinematics.getFy(), kinematics.getFz(), b.getBx(),
                b.getBy(), b.getBz(), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param position   Earth position resolved in NED coordinates.
     * @param year       a year expressed in decimal value.
     * @param kinematics body kinematics containing measured specific force.
     * @param b          measured body magnetic flux density.
     * @return a coordinate transformation containing body attitude.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public CoordinateTransformation getAttitude(
            final NEDPosition position,
            final double year,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b) {
        return getAttitude(position.getLatitude(), position.getLongitude(),
                position.getHeight(), year, kinematics.getFx(),
                kinematics.getFy(), kinematics.getFz(), b.getBx(),
                b.getBy(), b.getBz());
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param position   Earth position resolved in NED coordinates.
     * @param calendar   a calendar indicating a given timestamp.
     * @param kinematics body kinematics containing measured specific force.
     * @param b          measured body magnetic flux density.
     * @param result     instance where body attitude will be stored.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public void getAttitude(
            final NEDPosition position,
            final GregorianCalendar calendar,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final CoordinateTransformation result) {
        getAttitude(position.getLatitude(), position.getLongitude(),
                position.getHeight(), calendar, kinematics.getFx(),
                kinematics.getFy(), kinematics.getFz(), b.getBx(),
                b.getBy(), b.getBz(), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param position   Earth position resolved in NED coordinates.
     * @param calendar   a calendar indicating a given timestamp.
     * @param kinematics body kinematics containing measured specific force.
     * @param b          measured body magnetic flux density.
     * @return a coordinate transformation containing body attitude.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public CoordinateTransformation getAttitude(
            final NEDPosition position,
            final GregorianCalendar calendar,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b) {
        return getAttitude(position.getLatitude(), position.getLongitude(),
                position.getHeight(), calendar, kinematics.getFx(),
                kinematics.getFy(), kinematics.getFz(), b.getBx(),
                b.getBy(), b.getBz());
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param position   Earth position resolved in NED coordinates.
     * @param timestamp  a timestamp.
     * @param kinematics body kinematics containing measured specific force.
     * @param b          measured body magnetic flux density.
     * @param result     instance where body attitude will be stored.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public void getAttitude(
            final NEDPosition position,
            final Date timestamp,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final CoordinateTransformation result) {
        getAttitude(position.getLatitude(), position.getLongitude(),
                position.getHeight(), timestamp, kinematics.getFx(),
                kinematics.getFy(), kinematics.getFz(), b.getBx(),
                b.getBy(), b.getBz(), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a given World Magnetic Model (WMM)
     * to estimate the magnetic declination angle at a given Earth position
     * and timestamp.
     * Besides, a leveling algorithm is used to estimate roll and pitch
     * attitude angles, which can later be used to estimate yaw attude angle
     * with estimated declination angle and provided magnetometer measurements.
     *
     * @param position   Earth position resolved in NED coordinates.
     * @param timestamp  a timestamp.
     * @param kinematics body kinematics containing measured specific force.
     * @param b          measured body magnetic flux density.
     * @return a coordinate transformation containing body attitude.
     * @see LevelingEstimator2
     * @see WMMEarthMagneticFluxDensityEstimator
     */
    public CoordinateTransformation getAttitude(
            final NEDPosition position,
            final Date timestamp,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b) {
        return getAttitude(position.getLatitude(), position.getLongitude(),
                position.getHeight(), timestamp, kinematics.getFx(),
                kinematics.getFy(), kinematics.getFz(), b.getBx(),
                b.getBy(), b.getBz());
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     *
     * @param fx          x-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param fy          y-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param fz          z-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param result      instance where body attitude will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static void getAttitude(
            final double fx, final double fy, final double fz,
            final double bx, final double by, final double bz,
            final double declination,
            final CoordinateTransformation result) {
        result.setSourceType(FrameType.LOCAL_NAVIGATION_FRAME);
        result.setDestinationType(FrameType.BODY_FRAME);

        final double roll = LevelingEstimator.getRoll(fy, fz);
        final double pitch = LevelingEstimator.getPitch(fx, fy, fz);
        final double yaw = getYaw(bx, by, bz, declination, roll, pitch);

        result.setEulerAngles(roll, pitch, yaw);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     *
     * @param fx          x-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param fy          y-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param fz          z-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @return a coordinate transformation containing body attitude.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static CoordinateTransformation getAttitude(
            final double fx, final double fy, final double fz,
            final double bx, final double by, final double bz,
            final double declination) {

        final double roll = LevelingEstimator.getRoll(fy, fz);
        final double pitch = LevelingEstimator.getPitch(fx, fy, fz);
        final double yaw = getYaw(bx, by, bz, declination, roll, pitch);

        return new CoordinateTransformation(roll, pitch, yaw,
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     *
     * @param fx          x-coordinate of measured body specific force.
     * @param fy          y-coordinate of measured body specific force.
     * @param fz          z-coordinate of measured body specific force.
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param result      instance where body attitude will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static void getAttitude(
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final double bx, final double by, final double bz,
            final Angle declination,
            final CoordinateTransformation result) {
        getAttitude(convertAcceleration(fx), convertAcceleration(fy),
                convertAcceleration(fz), bx, by, bz,
                convertAngle(declination), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     *
     * @param fx          x-coordinate of measured body specific force.
     * @param fy          y-coordinate of measured body specific force.
     * @param fz          z-coordinate of measured body specific force.
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @return a coordinate transformation containing body attitude.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static CoordinateTransformation getAttitude(
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final double bx, final double by, final double bz,
            final Angle declination) {
        return getAttitude(convertAcceleration(fx), convertAcceleration(fy),
                convertAcceleration(fz), bx, by, bz,
                convertAngle(declination));
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     *
     * @param kinematics  body kinematics containing measured specific force.
     * @param b           measured body magnetic flux density.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param result      instance where body attitude will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static void getAttitude(
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final double declination,
            final CoordinateTransformation result) {
        getAttitude(kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                b.getBx(), b.getBy(), b.getBz(), declination, result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     *
     * @param kinematics  body kinematics containing measured specific force.
     * @param b           measured body magnetic flux density.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @return a coordinate transformation containing body attitude.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static CoordinateTransformation getAttitude(
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final double declination) {
        return getAttitude(kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz(), b.getBx(), b.getBy(), b.getBz(),
                declination);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     *
     * @param kinematics  body kinematics containing measured specific force.
     * @param b           measured body magnetic flux density.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param result      instance where body attitude will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static void getAttitude(
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final Angle declination,
            final CoordinateTransformation result) {
        getAttitude(kinematics, b, convertAngle(declination), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     *
     * @param kinematics  body kinematics containing measured specific force.
     * @param b           measured body magnetic flux density.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @return a coordinate transformation containing body attitude.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static CoordinateTransformation getAttitude(
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final Angle declination) {
        return getAttitude(kinematics, b, convertAngle(declination));
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     * Because latitude and height are also known, those are taken into
     * account considering actual Earth shape to obtain a slightly more
     * accurate estimation of roll and pitch attitude angles (and thus also
     * a more accurate yaw angle estimation).
     *
     * @param latitude    device latitude expressed in radians (rad).
     * @param height      device height expressed in meters (m).
     * @param fx          x-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param fy          y-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param fz          z-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param result      instance where body attitude will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator2
     */
    public static void getAttitude(
            final double latitude, final double height,
            final double fx, final double fy, final double fz,
            final double bx, final double by, final double bz,
            final double declination,
            final CoordinateTransformation result) {

        LevelingEstimator2.getPartialAttitude(latitude, height,
                fx, fy, fz, result);

        // fix yaw angle
        final double roll = result.getRollEulerAngle();
        final double pitch = result.getPitchEulerAngle();
        final double yaw = getYaw(bx, by, bz, declination, roll, pitch);

        result.setEulerAngles(roll, pitch, yaw);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     * Because latitude and height are also known, those are taken into
     * account considering actual Earth shape to obtain a slightly more
     * accurate estimation of roll and pitch attitude angles (and thus also
     * a more accurate yaw angle estimation).
     *
     * @param latitude    device latitude expressed in radians (rad).
     * @param height      device height expressed in meters (m).
     * @param fx          x-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param fy          y-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param fz          z-coordinate of measured body specific force
     *                    expressed in meters per squared second (m/s^2).
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @return a coordinate transformation containing body attitude.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator2
     */
    public static CoordinateTransformation getAttitude(
            final double latitude, final double height,
            final double fx, final double fy, final double fz,
            final double bx, final double by, final double bz,
            final double declination) {

        final CoordinateTransformation result = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
        getAttitude(latitude, height, fx, fy, fz, bx, by, bz, declination,
                result);
        return result;
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     * Because latitude and height are also known, those are taken into
     * account considering actual Earth shape to obtain a slightly more
     * accurate estimation of roll and pitch attitude angles (and thus also
     * a more accurate yaw angle estimation).
     *
     * @param latitude    device latitude.
     * @param height      device height.
     * @param fx          x-coordinate of measured body specific force.
     * @param fy          y-coordinate of measured body specific force.
     * @param fz          z-coordinate of measured body specific force.
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param result      instance where body attitude will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator2
     */
    public static void getAttitude(
            final Angle latitude, final Distance height,
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final double bx, final double by, final double bz,
            final Angle declination,
            final CoordinateTransformation result) {
        getAttitude(convertAngle(latitude), convertDistance(height),
                convertAcceleration(fx), convertAcceleration(fy),
                convertAcceleration(fz), bx, by, bz,
                convertAngle(declination), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     * Because latitude and height are also known, those are taken into
     * account considering actual Earth shape to obtain a slightly more
     * accurate estimation of roll and pitch attitude angles (and thus also
     * a more accurate yaw angle estimation).
     *
     * @param latitude    device latitude.
     * @param height      device height.
     * @param fx          x-coordinate of measured body specific force.
     * @param fy          y-coordinate of measured body specific force.
     * @param fz          z-coordinate of measured body specific force.
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @return a coordinate transformation containing body attitude.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator2
     */
    public static CoordinateTransformation getAttitude(
            final Angle latitude, final Distance height,
            final Acceleration fx, final Acceleration fy, final Acceleration fz,
            final double bx, final double by, final double bz,
            final Angle declination) {
        return getAttitude(convertAngle(latitude), convertDistance(height),
                convertAcceleration(fx), convertAcceleration(fy),
                convertAcceleration(fz), bx, by, bz,
                convertAngle(declination));
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     * Because Earth position is also known, it is taken into
     * account considering actual Earth shape to obtain a slightly more
     * accurate estimation of roll and pitch attitude angles (and thus also
     * a more accurate yaw angle estimation).
     *
     * @param position    Earth position expressed in NED coordinates.
     * @param kinematics  body kinematics containing measured specific force.
     * @param b           measured body magnetic flux density.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param result      instance where body attitude will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator2
     */
    public static void getAttitude(
            final NEDPosition position,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final double declination,
            final CoordinateTransformation result) {
        getAttitude(position.getLatitude(), position.getHeight(),
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                b.getBx(), b.getBy(), b.getBz(), declination,
                result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     * Because Earth position is also known, it is taken into
     * account considering actual Earth shape to obtain a slightly more
     * accurate estimation of roll and pitch attitude angles (and thus also
     * a more accurate yaw angle estimation).
     *
     * @param position    Earth position expressed in NED coordinates.
     * @param kinematics  body kinematics containing measured specific force.
     * @param b           measured body magnetic flux density.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @return a coordinate transformation containing body attitude.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator2
     */
    public static CoordinateTransformation getAttitude(
            final NEDPosition position,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final double declination) {
        return getAttitude(position.getLatitude(), position.getHeight(),
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                b.getBx(), b.getBy(), b.getBz(), declination);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     * Because Earth position is also known, it is taken into
     * account considering actual Earth shape to obtain a slightly more
     * accurate estimation of roll and pitch attitude angles (and thus also
     * a more accurate yaw angle estimation).
     *
     * @param position    Earth position expressed in NED coordinates.
     * @param kinematics  body kinematics containing measured specific force.
     * @param b           measured body magnetic flux density.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param result      instance where body attitude will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator2
     */
    public static void getAttitude(
            final NEDPosition position,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final Angle declination,
            final CoordinateTransformation result) {
        getAttitude(position, kinematics, b, convertAngle(declination),
                result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     * Internally this implementation uses a leveling algorithm to estimate
     * roll and pitch attitude angles, which can later be used to find
     * yaw attitude angle along with magnetometer measurements.
     * Because Earth position is also known, it is taken into
     * account considering actual Earth shape to obtain a slightly more
     * accurate estimation of roll and pitch attitude angles (and thus also
     * a more accurate yaw angle estimation).
     *
     * @param position    Earth position expressed in NED coordinates.
     * @param kinematics  body kinematics containing measured specific force.
     * @param b           measured body magnetic flux density.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @return a coordinate transformation containing body attitude.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     * @see LevelingEstimator
     */
    public static CoordinateTransformation getAttitude(
            final NEDPosition position,
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity b,
            final Angle declination) {
        return getAttitude(position, kinematics, b,
                convertAngle(declination));
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param bx    x coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param by    y coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param bz    z coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param roll  known body roll angle expressed in radians (rad).
     * @param pitch known body pitch angle expressed in radians (rad).
     * @return magnetic heading expressed in radians (rad).
     */
    public static double getMagneticHeading(
            final double bx, final double by, final double bz,
            final double roll, final double pitch) {

        final double sinRoll = Math.sin(roll);
        final double cosRoll = Math.cos(roll);

        final double sinPitch = Math.sin(pitch);
        final double cosPitch = Math.cos(pitch);

        final double tmp1 = -by * cosRoll + bz * sinRoll;
        final double tmp2 = bx * cosPitch + by * sinRoll * sinPitch
                + bz * cosRoll * sinPitch;

        return Math.atan2(tmp1, tmp2);
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param b     measured magnetic flux density resolved around body axes.
     * @param roll  known body roll angle expressed in radians (rad).
     * @param pitch known body pitch angle expressed in radians (rad).
     * @return magnetic heading expressed in radians (rad).
     */
    public static double getMagneticHeading(
            final BodyMagneticFluxDensity b,
            final double roll, final double pitch) {
        return getMagneticHeading(b.getBx(), b.getBy(), b.getBz(),
                roll, pitch);
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param bx    x coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param by    y coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param bz    z coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param roll  known body roll angle.
     * @param pitch known body pitch angle.
     * @return magnetic heading expressed in radians (rad).
     */
    public static double getMagneticHeading(
            final double bx, final double by, final double bz,
            final Angle roll, final Angle pitch) {
        return getMagneticHeading(bx, by, bz, convertAngle(roll),
                convertAngle(pitch));
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param b     measured magnetic flux density resolved around body axes.
     * @param roll  known body roll angle.
     * @param pitch known body pitch angle.
     * @return magnetic heading expressed in radians (rad).
     */
    public static double getMagneticHeading(
            final BodyMagneticFluxDensity b,
            final Angle roll, final Angle pitch) {
        return getMagneticHeading(b, convertAngle(roll),
                convertAngle(pitch));
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param bx     x coordinate of measured magnetic flux density resolved
     *               around body axes and expressed in Teslas (T).
     * @param by     y coordinate of measured magnetic flux density resolved
     *               around body axes and expressed in Teslas (T).
     * @param bz     z coordinate of measured magnetic flux density resolved
     *               around body axes and expressed in Teslas (T).
     * @param roll   known body roll angle expressed in radians (rad).
     * @param pitch  known body pitch angle expressed in radians (rad).
     * @param result instance where magnetic heading will be stored.
     */
    public static void getMagneticHeadingAsAngle(
            final double bx, final double by, final double bz,
            final double roll, final double pitch, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getMagneticHeading(bx, by, bz, roll, pitch));
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param bx    x coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param by    y coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param bz    z coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param roll  known body roll angle expressed in radians (rad).
     * @param pitch known body pitch angle expressed in radians (rad).
     * @return magnetic heading.
     */
    public static Angle getMagneticHeadingAsAngle(
            final double bx, final double by, final double bz,
            final double roll, final double pitch) {
        return new Angle(getMagneticHeading(bx, by, bz, roll, pitch),
                AngleUnit.RADIANS);
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param b      measured magnetic flux density resolved around body axes.
     * @param roll   known body roll angle expressed in radians (rad).
     * @param pitch  known body pitch angle expressed in radians (rad).
     * @param result instance where magnetic heading will be stored.
     */
    public static void getMagneticHeadingAsAngle(
            final BodyMagneticFluxDensity b,
            final double roll, final double pitch, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getMagneticHeading(b, roll, pitch));
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param b     measured magnetic flux density resolved around body axes.
     * @param roll  known body roll angle expressed in radians (rad).
     * @param pitch known body pitch angle expressed in radians (rad).
     * @return magnetic heading.
     */
    public static Angle getMagneticHeadingAsAngle(
            final BodyMagneticFluxDensity b,
            final double roll, final double pitch) {
        return new Angle(getMagneticHeading(b, roll, pitch),
                AngleUnit.RADIANS);
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param bx     x coordinate of measured magnetic flux density resolved
     *               around body axes and expressed in Teslas (T).
     * @param by     y coordinate of measured magnetic flux density resolved
     *               around body axes and expressed in Teslas (T).
     * @param bz     z coordinate of measured magnetic flux density resolved
     *               around body axes and expressed in Teslas (T).
     * @param roll   known body roll angle.
     * @param pitch  known body pitch angle.
     * @param result instance where magnetic heading will be stored.
     */
    public static void getMagneticHeadingAsAngle(
            final double bx, final double by, final double bz,
            final Angle roll, final Angle pitch, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getMagneticHeading(bx, by, bz, roll, pitch));
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param bx    x coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param by    y coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param bz    z coordinate of measured magnetic flux density resolved
     *              around body axes and expressed in Teslas (T).
     * @param roll  known body roll angle.
     * @param pitch known body pitch angle.
     * @return magnetic heading.
     */
    public static Angle getMagneticHeadingAsAngle(
            final double bx, final double by, final double bz,
            final Angle roll, final Angle pitch) {
        return new Angle(getMagneticHeading(bx, by, bz, roll, pitch),
                AngleUnit.RADIANS);
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param b      measured magnetic flux density resolved around body axes.
     * @param roll   known body roll angle.
     * @param pitch  known body pitch angle.
     * @param result instance where magnetic heading will be stored.
     */
    public static void getMagneticHeadingAsAngle(
            final BodyMagneticFluxDensity b,
            final Angle roll, final Angle pitch, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getMagneticHeading(b, roll, pitch));
    }

    /**
     * Returns magnetic heading (angle respect Earth magnetic north).
     *
     * @param b     measured magnetic flux density resolved around body axes.
     * @param roll  known body roll angle.
     * @param pitch known body pitch angle.
     * @return magnetic heading.
     */
    public static Angle getMagneticHeadingAsAngle(
            final BodyMagneticFluxDensity b,
            final Angle roll, final Angle pitch) {
        return new Angle(getMagneticHeading(b, roll, pitch),
                AngleUnit.RADIANS);
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle expressed in radians (rad).
     * @param pitch       known body pitch angle expressed in radians (rad).
     * @return body yaw angle (a.k.a true heading) expressed in radians (rad).
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static double getYaw(
            final double bx, final double by, final double bz,
            final double declination, final double roll, final double pitch) {
        return getMagneticHeading(bx, by, bz, roll, pitch) + declination;
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param b           measured magnetic flux density resolved around body axes.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle expressed in radians (rad).
     * @param pitch       known body pitch angle expressed in radians (rad).
     * @return body yaw angle (a.k.a true heading) expressed in radians (rad).
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static double getYaw(
            final BodyMagneticFluxDensity b, final double declination,
            final double roll, final double pitch) {
        return getYaw(b.getBx(), b.getBy(), b.getBz(), declination,
                roll, pitch);
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle.
     * @param pitch       known body pitch angle.
     * @return body yaw angle (a.k.a true heading) expressed in radians (rad).
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static double getYaw(
            final double bx, final double by, final double bz,
            final Angle declination, final Angle roll, final Angle pitch) {
        return getYaw(bx, by, bz, convertAngle(declination),
                convertAngle(roll), convertAngle(pitch));
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param b           measured magnetic flux density resolved around body axes.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle.
     * @param pitch       known body pitch angle.
     * @return body yaw angle (a.k.a true heading) expressed in radians (rad).
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static double getYaw(
            final BodyMagneticFluxDensity b, final Angle declination,
            final Angle roll, final Angle pitch) {
        return getYaw(b, convertAngle(declination), convertAngle(roll),
                convertAngle(pitch));
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle expressed in radians (rad).
     * @param pitch       known body pitch angle expressed in radians (rad).
     * @param result      instance where body yaw angle (a.k.a. true heading)
     *                    will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static void getYawAsAngle(
            final double bx, final double by, final double bz,
            final double declination, final double roll, final double pitch,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(bx, by, bz, declination, roll, pitch));
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle expressed in radians (rad).
     * @param pitch       known body pitch angle expressed in radians (rad).
     * @return body yaw angle (a.k.a. true heading).
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static Angle getYawAsAngle(
            final double bx, final double by, final double bz,
            final double declination, final double roll, final double pitch) {
        return new Angle(getYaw(bx, by, bz, declination, roll, pitch),
                AngleUnit.RADIANS);
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param b           measured magnetic flux density resolved around body axes.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle expressed in radians (rad).
     * @param pitch       known body pitch angle expressed in radians (rad).
     * @param result      instance where body yaw angle (a.k.a. true heading)
     *                    will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static void getYawAsAngle(
            final BodyMagneticFluxDensity b, final double declination,
            final double roll, final double pitch, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(b, declination, roll, pitch));
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param b           measured magnetic flux density resolved around body axes.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant expressed in
     *                    radians. Declination angle changes depending on
     *                    instant and location. Declination angle can be
     *                    obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle expressed in radians (rad).
     * @param pitch       known body pitch angle expressed in radians (rad).
     * @return body yaw angle (a.k.a. true heading).
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static Angle getYawAsAngle(
            final BodyMagneticFluxDensity b, final double declination,
            final double roll, final double pitch) {
        return new Angle(getYaw(b, declination, roll, pitch),
                AngleUnit.RADIANS);
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle.
     * @param pitch       known body pitch angle.
     * @param result      instance where body yaw angle (a.k.a. true heading)
     *                    will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static void getYawAsAngle(
            final double bx, final double by, final double bz,
            final Angle declination, final Angle roll, final Angle pitch,
            final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(bx, by, bz, declination, roll, pitch));
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param bx          x coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param by          y coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param bz          z coordinate of measured magnetic flux density resolved
     *                    around body axes and expressed in Teslas (T).
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle.
     * @param pitch       known body pitch angle.
     * @return body yaw angle (a.k.a. true heading).
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static Angle getYawAsAngle(
            final double bx, final double by, final double bz,
            final Angle declination, final Angle roll, final Angle pitch) {
        return new Angle(getYaw(bx, by, bz, declination, roll, pitch),
                AngleUnit.RADIANS);
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param b           measured magnetic flux density resolved around body axes.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle.
     * @param pitch       known body pitch angle.
     * @param result      instance where body yaw angle (a.k.a. true heading)
     *                    will be stored.
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static void getYawAsAngle(
            final BodyMagneticFluxDensity b, final Angle declination,
            final Angle roll, final Angle pitch, final Angle result) {
        result.setUnit(AngleUnit.RADIANS);
        result.setValue(getYaw(b, declination, roll, pitch));
    }

    /**
     * Returns body yaw angle resolved around NED frame.
     * Yaw angle represents the true heading, and indicates the angle respect
     * the true geographic north.
     *
     * @param b           measured magnetic flux density resolved around body axes.
     * @param declination declination angle of Earth magnetic flux density
     *                    at current Earth position and instant. Declination
     *                    angle changes depending on instant and location.
     *                    Declination angle can be obtained through
     *                    {@link WMMEarthMagneticFluxDensityEstimator#getDeclination(NEDPosition, Date)}
     *                    or using {@link EarthMagneticFluxDensityEstimator#getDeclination(NEDMagneticFluxDensity)}
     * @param roll        known body roll angle.
     * @param pitch       known body pitch angle.
     * @return body yaw angle (a.k.a. true heading).
     * @see WMMEarthMagneticFluxDensityEstimator
     * @see EarthMagneticFluxDensityEstimator
     */
    public static Angle getYawAsAngle(
            final BodyMagneticFluxDensity b, final Angle declination,
            final Angle roll, final Angle pitch) {
        return new Angle(getYaw(b, declination, roll, pitch),
                AngleUnit.RADIANS);
    }

    /**
     * Converts a given distance instance into meters.
     *
     * @param distance distance to be converted.
     * @return converted value expressed in meters.
     */
    private static double convertDistance(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(),
                distance.getUnit(), DistanceUnit.METER);
    }

    /**
     * Converts a given angle instance into radians.
     *
     * @param angle angle to be converted.
     * @return converted value expressed in radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Converts an instance of acceleration to meters per squared second (m/s^2).
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted acceleration value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(
                acceleration.getValue().doubleValue(),
                acceleration.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }
}
