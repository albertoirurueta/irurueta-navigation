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
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimatorTest;
import com.irurueta.navigation.geodesic.wmm.WMMLoader;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Random;

import static org.junit.Assert.*;

public class AttitudeEstimatorTest {

    private static final String RESOURCE = "wmm.cof";

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final double LARGE_ABSOLUTE_ERROR = 4e-4;
    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    private static final int TIMES = 100;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1,
                0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31,
                23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    @Test
    public void testConstructor() throws IOException {
        final AttitudeEstimator estimator1 = new AttitudeEstimator();

        assertNotNull(estimator1);

        final WorldMagneticModel model = WMMLoader.loadFromResource(RESOURCE);
        final AttitudeEstimator estimator2 = new AttitudeEstimator(model);

        assertNotNull(estimator2);
    }

    @Test
    public void testGetAttitude1() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, year);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        estimator.getAttitude(latitude, longitude, height, year,
                fx, fy, fz, bx, by, bz, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude2() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, year);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final AttitudeEstimator estimator = new AttitudeEstimator();
        final CoordinateTransformation result = estimator.getAttitude(
                latitude, longitude, height, year,
                fx, fy, fz, bx, by, bz);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude3() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final GregorianCalendar calendar = createCalendar(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, calendar);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        estimator.getAttitude(latitude, longitude, height, calendar,
                fx, fy, fz, bx, by, bz, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude4() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final GregorianCalendar calendar = createCalendar(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, calendar);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final AttitudeEstimator estimator = new AttitudeEstimator();
        final CoordinateTransformation result = estimator.getAttitude(
                latitude, longitude, height, calendar,
                fx, fy, fz, bx, by, bz);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude5() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        estimator.getAttitude(latitude, longitude, height, date,
                fx, fy, fz, bx, by, bz, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude6() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final AttitudeEstimator estimator = new AttitudeEstimator();
        final CoordinateTransformation result = estimator.getAttitude(
                latitude, longitude, height, date,
                fx, fy, fz, bx, by, bz);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude7() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, year);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        estimator.getAttitude(position, year,
                kinematics, b, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude8() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, year);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        final CoordinateTransformation result = estimator.getAttitude(
                position, year, kinematics, b);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude9() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final GregorianCalendar calendar = createCalendar(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, calendar);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        estimator.getAttitude(position, calendar,
                kinematics, b, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude10() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final GregorianCalendar calendar = createCalendar(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, calendar);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        final CoordinateTransformation result = estimator.getAttitude(
                position, calendar, kinematics, b);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude11() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        estimator.getAttitude(position, date,
                kinematics, b, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitude12() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final AttitudeEstimator estimator = new AttitudeEstimator();
        final CoordinateTransformation result = estimator.getAttitude(
                position, date, kinematics, b);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic1() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final double declination = wmmEstimator.getDeclination(
                position, date);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(fx, fy, fz, bx, by, bz,
                declination, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic2() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final double declination = wmmEstimator.getDeclination(
                position, date);

        final CoordinateTransformation result = AttitudeEstimator
                .getAttitude(fx, fy, fz, bx, by, bz, declination);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic3() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final Angle declination = wmmEstimator.getDeclinationAsAngle(
                position, date);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(fx, fy, fz, bx, by, bz,
                declination, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic4() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final Angle declination = wmmEstimator.getDeclinationAsAngle(
                position, date);

        final CoordinateTransformation result = AttitudeEstimator
                .getAttitude(fx, fy, fz, bx, by, bz, declination);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic5() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final double declination = wmmEstimator.getDeclination(
                position, date);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(kinematics, b,
                declination, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic6() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final double declination = wmmEstimator.getDeclination(
                position, date);

        final CoordinateTransformation result = AttitudeEstimator
                .getAttitude(kinematics, b, declination);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic7() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final Angle declination = wmmEstimator.getDeclinationAsAngle(
                position, date);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(kinematics, b,
                declination, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic8() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final Angle declination = wmmEstimator.getDeclinationAsAngle(
                position, date);

        final CoordinateTransformation result = AttitudeEstimator
                .getAttitude(kinematics, b, declination);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic9() throws IOException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {

            final NEDPosition position = createPosition();
            final double latitude = position.getLatitude();
            final double height = position.getHeight();

            final long timestamp = createTimestamp();
            final Date date = new Date(timestamp);

            // body attitude
            final UniformRandomizer randomizer =
                    new UniformRandomizer(new Random());
            final double roll1 = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES,
                            MAX_ANGLE_DEGREES));
            final double pitch1 = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES,
                            MAX_ANGLE_DEGREES));
            final double yaw1 = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES,
                            MAX_ANGLE_DEGREES));

            // attitude is expressed as rotation from local navigation frame
            // to body frame, since angles are measured on the device body
            final CoordinateTransformation bodyC = new CoordinateTransformation(
                    roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.BODY_FRAME);
            final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

            // obtain expected kinematics measure
            final BodyKinematics kinematics = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                            nedC, nedC,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            latitude, height,
                            latitude, height);
            final double fx = kinematics.getFx();
            final double fy = kinematics.getFy();
            final double fz = kinematics.getFz();

            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();
            final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                    position, date);
            final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                    .estimate(earthB, bodyC);
            final double bx = b.getBx();
            final double by = b.getBy();
            final double bz = b.getBz();

            final double declination = wmmEstimator.getDeclination(
                    position, date);

            final CoordinateTransformation result =
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME);
            AttitudeEstimator.getAttitude(latitude, height,
                    fx, fy, fz, bx, by, bz,
                    declination, result);

            // check
            final double roll2 = result.getRollEulerAngle();
            final double pitch2 = result.getPitchEulerAngle();
            final double yaw2 = result.getYawEulerAngle();

            if (Math.abs(roll1 - roll2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(pitch1 - pitch2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(yaw1 - yaw2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
            assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
            assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

            assertEquals(result.getSourceType(),
                    FrameType.LOCAL_NAVIGATION_FRAME);
            assertEquals(result.getDestinationType(),
                    FrameType.BODY_FRAME);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGetAttitudeStatic10() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final double fx = kinematics.getFx();
        final double fy = kinematics.getFy();
        final double fz = kinematics.getFz();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final double declination = wmmEstimator.getDeclination(
                position, date);

        final CoordinateTransformation result = AttitudeEstimator
                .getAttitude(latitude, height,
                fx, fy, fz, bx, by, bz, declination);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic11() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height,
                DistanceUnit.METER);

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);
        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final Angle declination = wmmEstimator.getDeclinationAsAngle(
                position, date);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(latitudeAngle, heightDistance,
                fx, fy, fz, bx, by, bz,
                declination, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic12() throws IOException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {

            final NEDPosition position = createPosition();
            final double latitude = position.getLatitude();
            final double height = position.getHeight();

            final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
            final Distance heightDistance = new Distance(height,
                    DistanceUnit.METER);

            final long timestamp = createTimestamp();
            final Date date = new Date(timestamp);

            // body attitude
            final UniformRandomizer randomizer =
                    new UniformRandomizer(new Random());
            final double roll1 = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES,
                            MAX_ANGLE_DEGREES));
            final double pitch1 = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES,
                            MAX_ANGLE_DEGREES));
            final double yaw1 = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES,
                            MAX_ANGLE_DEGREES));

            // attitude is expressed as rotation from local navigation frame
            // to body frame, since angles are measured on the device body
            final CoordinateTransformation bodyC = new CoordinateTransformation(
                    roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.BODY_FRAME);
            final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

            // obtain expected kinematics measure
            final BodyKinematics kinematics = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                            nedC, nedC,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            latitude, height,
                            latitude, height);
            final Acceleration fx = kinematics.getSpecificForceX();
            final Acceleration fy = kinematics.getSpecificForceY();
            final Acceleration fz = kinematics.getSpecificForceZ();

            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();
            final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                    position, date);
            final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                    .estimate(earthB, bodyC);
            final double bx = b.getBx();
            final double by = b.getBy();
            final double bz = b.getBz();

            final Angle declination = wmmEstimator.getDeclinationAsAngle(
                    position, date);

            final CoordinateTransformation result = AttitudeEstimator
                    .getAttitude(latitudeAngle, heightDistance,
                            fx, fy, fz, bx, by, bz, declination);

            // check
            final double roll2 = result.getRollEulerAngle();
            final double pitch2 = result.getPitchEulerAngle();
            final double yaw2 = result.getYawEulerAngle();

            if (Math.abs(roll1 - roll2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(pitch1 - pitch2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(yaw1 - yaw2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
            assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
            assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

            assertEquals(result.getSourceType(),
                    FrameType.LOCAL_NAVIGATION_FRAME);
            assertEquals(result.getDestinationType(),
                    FrameType.BODY_FRAME);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGetAttitudeStatic13() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final double declination = wmmEstimator.getDeclination(
                position, date);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(position, kinematics, b,
                declination, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic14() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final double declination = wmmEstimator.getDeclination(
                position, date);

        final CoordinateTransformation result = AttitudeEstimator
                .getAttitude(position, kinematics, b, declination);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic15() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final Angle declination = wmmEstimator.getDeclinationAsAngle(
                position, date);

        final CoordinateTransformation result =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(position, kinematics, b,
                declination, result);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetAttitudeStatic16() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double height = position.getHeight();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);

        final Angle declination = wmmEstimator.getDeclinationAsAngle(
                position, date);

        final CoordinateTransformation result = AttitudeEstimator
                .getAttitude(position, kinematics, b, declination);

        // check
        final double roll2 = result.getRollEulerAngle();
        final double pitch2 = result.getPitchEulerAngle();
        final double yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(result.getSourceType(),
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(result.getDestinationType(),
                FrameType.BODY_FRAME);
    }

    @Test
    public void testGetMagneticHeading() throws IOException {
        final NEDPosition position = createPosition();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double magneticHeading1 = yaw - declination;

        final double magneticHeading2 = AttitudeEstimator
                .getMagneticHeading(bx, by, bz, roll, pitch);
        final double magneticHeading3 = AttitudeEstimator
                .getMagneticHeading(b, roll, pitch);

        final Angle rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final Angle pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final double magneticHeading4 = AttitudeEstimator
                .getMagneticHeading(bx, by, bz, rollAngle, pitchAngle);
        final double magneticHeading5 = AttitudeEstimator
                .getMagneticHeading(b, rollAngle, pitchAngle);

        assertEquals(magneticHeading1, magneticHeading2, ABSOLUTE_ERROR);
        assertEquals(magneticHeading1, magneticHeading3, ABSOLUTE_ERROR);
        assertEquals(magneticHeading1, magneticHeading4, ABSOLUTE_ERROR);
        assertEquals(magneticHeading1, magneticHeading5, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetMagneticHeadingAsAngle() throws IOException {
        final NEDPosition position = createPosition();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double magneticHeading1 = yaw - declination;

        final Angle magneticHeading2 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getMagneticHeadingAsAngle(bx, by, bz, roll, pitch,
                magneticHeading2);
        final Angle magneticHeading3 = AttitudeEstimator
                .getMagneticHeadingAsAngle(bx, by, bz, roll, pitch);
        final Angle magneticHeading4 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getMagneticHeadingAsAngle(b, roll, pitch,
                magneticHeading4);
        final Angle magneticHeading5 = AttitudeEstimator
                .getMagneticHeadingAsAngle(b, roll, pitch);

        final Angle rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final Angle pitchAngle = new Angle(pitch, AngleUnit.RADIANS);

        final Angle magneticHeading6 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getMagneticHeadingAsAngle(bx, by, bz,
                rollAngle, pitchAngle, magneticHeading6);
        final Angle magneticHeading7 = AttitudeEstimator
                .getMagneticHeadingAsAngle(bx, by, bz,
                        rollAngle, pitchAngle);
        final Angle magneticHeading8 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getMagneticHeadingAsAngle(b,
                rollAngle, pitchAngle, magneticHeading8);
        final Angle magneticHeading9 = AttitudeEstimator
                .getMagneticHeadingAsAngle(b, rollAngle, pitchAngle);

        assertEquals(magneticHeading1,
                magneticHeading2.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(magneticHeading2.getUnit(), AngleUnit.RADIANS);
        assertEquals(magneticHeading2, magneticHeading3);
        assertEquals(magneticHeading2, magneticHeading4);
        assertEquals(magneticHeading2, magneticHeading5);
        assertEquals(magneticHeading2, magneticHeading6);
        assertEquals(magneticHeading2, magneticHeading7);
        assertEquals(magneticHeading2, magneticHeading8);
        assertEquals(magneticHeading2, magneticHeading9);
    }

    @Test
    public void testGetYaw() throws IOException {
        final NEDPosition position = createPosition();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final double declination = wmmEstimator.getDeclination(
                position, date);

        double yaw2 = AttitudeEstimator.getYaw(bx, by, bz,
                declination, roll, pitch);
        final double yaw3 = AttitudeEstimator.getYaw(b, declination,
                roll, pitch);

        final Angle rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final Angle pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final Angle declinationAngle = new Angle(declination,
                AngleUnit.RADIANS);
        final double yaw4 = AttitudeEstimator.getYaw(bx, by, bz,
                declinationAngle, rollAngle, pitchAngle);
        final double yaw5 = AttitudeEstimator.getYaw(b, declinationAngle,
                rollAngle, pitchAngle);

        if (Math.abs(yaw2) > Math.PI) {
            yaw2 = 2.0 * Math.PI + yaw2;
        }
        assertEquals(yaw1, yaw2, ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3, ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw4, ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw5, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetYawAsAngle() throws IOException {
        final NEDPosition position = createPosition();

        final long timestamp = createTimestamp();
        final Date date = new Date(timestamp);

        // body attitude
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, date);
        final BodyMagneticFluxDensity b = BodyMagneticFluxDensityEstimator
                .estimate(earthB, bodyC);
        final double bx = b.getBx();
        final double by = b.getBy();
        final double bz = b.getBz();

        final double declination = wmmEstimator.getDeclination(
                position, date);

        final Angle yaw2 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getYawAsAngle(bx, by, bz, declination,
                roll, pitch, yaw2);
        final Angle yaw3 = AttitudeEstimator.getYawAsAngle(bx, by, bz,
                declination, roll, pitch);
        final Angle yaw4 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getYawAsAngle(b, declination, roll, pitch, yaw4);
        final Angle yaw5 = AttitudeEstimator.getYawAsAngle(b, declination,
                roll, pitch);

        final Angle rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final Angle pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final Angle declinationAngle = new Angle(declination,
                AngleUnit.RADIANS);
        final Angle yaw6 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getYawAsAngle(bx, by, bz, declinationAngle,
                rollAngle, pitchAngle, yaw6);
        final Angle yaw7 = AttitudeEstimator.getYawAsAngle(bx, by, bz,
                declinationAngle, rollAngle, pitchAngle);
        final Angle yaw8 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getYawAsAngle(b, declinationAngle,
                rollAngle, pitchAngle, yaw8);
        final Angle yaw9 = AttitudeEstimator.getYawAsAngle(b,
                declinationAngle, rollAngle, pitchAngle);

        assertEquals(yaw1, yaw2.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(yaw2.getUnit(), AngleUnit.RADIANS);
        assertEquals(yaw2, yaw3);
        assertEquals(yaw2, yaw4);
        assertEquals(yaw2, yaw5);
        assertEquals(yaw2, yaw6);
        assertEquals(yaw2, yaw7);
        assertEquals(yaw2, yaw8);
        assertEquals(yaw2, yaw9);
    }

    private static NEDPosition createPosition() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(
                MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        return randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }

    private static GregorianCalendar createCalendar(final long timestamp) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestamp);
        return calendar;
    }

    private static double createYear(final GregorianCalendar calendar) {
        return WMMEarthMagneticFluxDensityEstimatorTest.createYear(calendar);
    }
}
