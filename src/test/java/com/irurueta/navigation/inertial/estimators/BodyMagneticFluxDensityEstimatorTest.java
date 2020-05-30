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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.geodesic.wmm.EarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.Random;

import static org.junit.Assert.*;

public class BodyMagneticFluxDensityEstimatorTest {

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1,
                0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31,
                23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    public void testDeclinationAndDip() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double declination1 = wmmEstimator.getDeclination(
                position, date);
        final double dip1 = wmmEstimator.getDip(position, date);

        final NEDMagneticFluxDensity b = wmmEstimator.estimate(
                position, date);

        // test that declination and dip values match for both WMM
        // and Earth magnetic flux density estimator.
        final double declination2 = EarthMagneticFluxDensityEstimator
                .getDeclination(b);
        final double dip2 = EarthMagneticFluxDensityEstimator.getDip(b);

        assertEquals(declination1, declination2, ABSOLUTE_ERROR);
        assertEquals(dip1, dip2, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimate1() throws IOException, WrongSizeException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final NEDMagneticFluxDensity bEarth = wmmEstimator.estimate(
                position, date);

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final BodyMagneticFluxDensity result1 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(
                magnitude, declination, dip, roll, pitch, yaw,
                result1);

        final Matrix b = bEarth.asMatrix();
        final Matrix cnb = c.getMatrix();
        final Matrix expected = cnb.multiplyAndReturnNew(b);

        assertArrayEquals(expected.getBuffer(), result1.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimate2() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final BodyMagneticFluxDensity result1 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(
                magnitude, declination, dip, roll, pitch, yaw,
                result1);
        final BodyMagneticFluxDensity result2 =
                BodyMagneticFluxDensityEstimator.estimate(
                        magnitude, declination, dip, roll, pitch, yaw);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate3() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final BodyMagneticFluxDensity result1 =
                BodyMagneticFluxDensityEstimator.estimate(
                        magnitude, declination, dip, roll, pitch, yaw);
        final BodyMagneticFluxDensity result2 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(
                magnitude, declination, dip, c, result2);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate4() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final CoordinateTransformation c = createAttitude();

        final BodyMagneticFluxDensity result1 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(
                magnitude, declination, dip, c, result1);
        final BodyMagneticFluxDensity result2 =
                BodyMagneticFluxDensityEstimator.estimate(
                        magnitude, declination, dip, c);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate5() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final NEDMagneticFluxDensity bEarth = wmmEstimator.estimate(
                position, date);

        final BodyMagneticFluxDensity result1 =
                BodyMagneticFluxDensityEstimator.estimate(
                        magnitude, declination, dip, c);
        final BodyMagneticFluxDensity result2 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth,
                roll, pitch, yaw, result2);

        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));
    }

    @Test
    public void testEstimate6() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final NEDMagneticFluxDensity bEarth = wmmEstimator.estimate(
                position, date);

        final BodyMagneticFluxDensity result1 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth,
                roll, pitch, yaw, result1);
        final BodyMagneticFluxDensity result2 =
                BodyMagneticFluxDensityEstimator.estimate(bEarth,
                        roll, pitch, yaw);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate7() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final NEDMagneticFluxDensity bEarth = wmmEstimator.estimate(
                position, date);

        final BodyMagneticFluxDensity result1 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth,
                roll, pitch, yaw, result1);
        final BodyMagneticFluxDensity result2 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth, c, result2);

        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));
    }

    @Test
    public void testEstimate8() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final CoordinateTransformation c = createAttitude();

        final NEDMagneticFluxDensity bEarth = wmmEstimator.estimate(
                position, date);

        final BodyMagneticFluxDensity result1 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth, c, result1);
        final BodyMagneticFluxDensity result2 =
                BodyMagneticFluxDensityEstimator.estimate(bEarth, c);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate9() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final Angle declinationAngle = new Angle(declination,
                AngleUnit.RADIANS);
        final Angle dipAngle = new Angle(dip, AngleUnit.RADIANS);
        final Angle rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final Angle pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final Angle yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        final BodyMagneticFluxDensity result1 =
                BodyMagneticFluxDensityEstimator.estimate(
                        magnitude, declination, dip, roll, pitch, yaw);
        final BodyMagneticFluxDensity result2 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(magnitude, declinationAngle,
                dipAngle, rollAngle, pitchAngle, yawAngle, result2);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate10() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final Angle declinationAngle = new Angle(declination,
                AngleUnit.RADIANS);
        final Angle dipAngle = new Angle(dip, AngleUnit.RADIANS);
        final Angle rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final Angle pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final Angle yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        final BodyMagneticFluxDensity result1 =
                BodyMagneticFluxDensityEstimator.estimate(
                        magnitude, declination, dip, roll, pitch, yaw);
        final BodyMagneticFluxDensity result2 =
                BodyMagneticFluxDensityEstimator.estimate(
                        magnitude, declinationAngle, dipAngle, rollAngle,
                        pitchAngle, yawAngle);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate11() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final CoordinateTransformation c = createAttitude();

        final Angle declinationAngle = new Angle(declination,
                AngleUnit.RADIANS);
        final Angle dipAngle = new Angle(dip, AngleUnit.RADIANS);

        final BodyMagneticFluxDensity result1 = BodyMagneticFluxDensityEstimator
                .estimate(magnitude, declination, dip, c);
        final BodyMagneticFluxDensity result2 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(magnitude,
                declinationAngle, dipAngle, c, result2);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate12() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final double magnitude = wmmEstimator.getIntensity(
                position, date);
        final double declination = wmmEstimator.getDeclination(
                position, date);
        final double dip = wmmEstimator.getDip(position, date);

        final CoordinateTransformation c = createAttitude();

        final Angle declinationAngle = new Angle(declination,
                AngleUnit.RADIANS);
        final Angle dipAngle = new Angle(dip, AngleUnit.RADIANS);

        final BodyMagneticFluxDensity result1 = BodyMagneticFluxDensityEstimator
                .estimate(magnitude, declination, dip, c);
        final BodyMagneticFluxDensity result2 =
                BodyMagneticFluxDensityEstimator.estimate(magnitude,
                        declinationAngle, dipAngle, c);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate13() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final NEDMagneticFluxDensity bEarth = wmmEstimator.estimate(
                position, date);

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final Angle rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final Angle pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final Angle yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        final BodyMagneticFluxDensity result1 =
                BodyMagneticFluxDensityEstimator.estimate(bEarth,
                        roll, pitch, yaw);
        final BodyMagneticFluxDensity result2 =
                new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth,
                rollAngle, pitchAngle, yawAngle, result2);

        assertEquals(result1, result2);
    }

    @Test
    public void testEstimate14() throws IOException {
        final NEDPosition position = createPosition();
        final Date date = createTimestamp();

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator
                = new WMMEarthMagneticFluxDensityEstimator();

        final NEDMagneticFluxDensity bEarth = wmmEstimator.estimate(
                position, date);

        final CoordinateTransformation c = createAttitude();
        final double roll = c.getRollEulerAngle();
        final double pitch = c.getPitchEulerAngle();
        final double yaw = c.getYawEulerAngle();

        final Angle rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final Angle pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final Angle yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        final BodyMagneticFluxDensity result1 =
                BodyMagneticFluxDensityEstimator.estimate(bEarth,
                        roll, pitch, yaw);
        final BodyMagneticFluxDensity result2 =
                BodyMagneticFluxDensityEstimator.estimate(bEarth,
                        rollAngle, pitchAngle, yawAngle);

        assertEquals(result1, result2);
    }


    private CoordinateTransformation createAttitude() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw,
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
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

    private static Date createTimestamp() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        return new Date(randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS));
    }
}
