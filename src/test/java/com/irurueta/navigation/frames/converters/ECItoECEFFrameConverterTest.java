/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.frames.converters;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ECItoECEFFrameConverterTest {

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_Z_VALUE = -50.0;
    private static final double MAX_Z_VALUE = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {
        assertEquals(ECItoECEFFrameConverter.EARTH_ROTATION_RATE, Constants.EARTH_ROTATION_RATE, 0.0);
    }

    @Test
    public void testConstructor() {
        final ECItoECEFFrameConverter converter = new ECItoECEFFrameConverter();

        // check
        assertEquals(converter.getSourceType(),
                FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        assertEquals(converter.getDestinationType(),
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
    }

    @Test
    public void testConvertAndReturnNewWithSecondsTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformation c = new CoordinateTransformation(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            final ECIFrame eciFrame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

            final ECItoECEFFrameConverter converter1 = new ECItoECEFFrameConverter();

            final ECEFFrame ecefFrame = converter1.convertAndReturnNew(TIME_INTERVAL_SECONDS, eciFrame1);

            // convert back to ECI
            final ECEFtoECIFrameConverter converter2 = new ECEFtoECIFrameConverter();

            final ECIFrame eciFrame2 = converter2.convertAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame);

            // check
            assertEquals(eciFrame1.getX(), eciFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getY(), eciFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getZ(), eciFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getVx(), eciFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVy(), eciFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVz(), eciFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getCoordinateTransformation().getSourceType(),
                    eciFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(eciFrame1.getCoordinateTransformation().getDestinationType(),
                    eciFrame2.getCoordinateTransformation().getDestinationType());

            final Quaternion q1 = new Quaternion();
            eciFrame1.getCoordinateTransformation().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            eciFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(eciFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testConvertWithSecondsTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformation c = new CoordinateTransformation(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            final ECIFrame eciFrame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

            final ECItoECEFFrameConverter converter1 = new ECItoECEFFrameConverter();

            final ECEFFrame ecefFrame = new ECEFFrame();
            converter1.convert(TIME_INTERVAL_SECONDS, eciFrame1, ecefFrame);

            // convert back to ECI
            final ECEFtoECIFrameConverter converter2 = new ECEFtoECIFrameConverter();

            final ECIFrame eciFrame2 = new ECIFrame();
            converter2.convert(TIME_INTERVAL_SECONDS, ecefFrame, eciFrame2);

            // check
            assertEquals(eciFrame1.getX(), eciFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getY(), eciFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getZ(), eciFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getVx(), eciFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVy(), eciFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVz(), eciFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getCoordinateTransformation().getSourceType(),
                    eciFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(eciFrame1.getCoordinateTransformation().getDestinationType(),
                    eciFrame2.getCoordinateTransformation().getDestinationType());

            final Quaternion q1 = new Quaternion();
            eciFrame1.getCoordinateTransformation().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            eciFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(eciFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testConvertECIToECEFAndReturnNewWithSecondsTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformation c = new CoordinateTransformation(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            final ECIFrame eciFrame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

            final ECEFFrame ecefFrame = ECItoECEFFrameConverter.convertECItoECEFAndReturnNew(
                    TIME_INTERVAL_SECONDS, eciFrame1);

            // convert back to ECI
            final ECIFrame eciFrame2 = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                    TIME_INTERVAL_SECONDS, ecefFrame);

            // check
            assertEquals(eciFrame1.getX(), eciFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getY(), eciFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getZ(), eciFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getVx(), eciFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVy(), eciFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVz(), eciFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getCoordinateTransformation().getSourceType(),
                    eciFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(eciFrame1.getCoordinateTransformation().getDestinationType(),
                    eciFrame2.getCoordinateTransformation().getDestinationType());

            final Quaternion q1 = new Quaternion();
            eciFrame1.getCoordinateTransformation().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            eciFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(eciFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testConvertECItoECEFWithSecondsTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformation c = new CoordinateTransformation(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            final ECIFrame eciFrame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

            final ECEFFrame ecefFrame = new ECEFFrame();
            ECItoECEFFrameConverter.convertECItoECEF(TIME_INTERVAL_SECONDS, eciFrame1, ecefFrame);

            // convert back to ECI
            final ECIFrame eciFrame2 = new ECIFrame();
            ECEFtoECIFrameConverter.convertECEFtoECI(TIME_INTERVAL_SECONDS, ecefFrame, eciFrame2);

            // check
            assertEquals(eciFrame1.getX(), eciFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getY(), eciFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getZ(), eciFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getVx(), eciFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVy(), eciFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVz(), eciFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getCoordinateTransformation().getSourceType(),
                    eciFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(eciFrame1.getCoordinateTransformation().getDestinationType(),
                    eciFrame2.getCoordinateTransformation().getDestinationType());

            final Quaternion q1 = new Quaternion();
            eciFrame1.getCoordinateTransformation().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            eciFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(eciFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testConvertAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformation c = new CoordinateTransformation(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            final ECIFrame eciFrame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

            final ECItoECEFFrameConverter converter1 = new ECItoECEFFrameConverter();

            final ECEFFrame ecefFrame = converter1.convertAndReturnNew(timeInterval, eciFrame1);

            // convert back to ECI
            final ECEFtoECIFrameConverter converter2 = new ECEFtoECIFrameConverter();

            final ECIFrame eciFrame2 = converter2.convertAndReturnNew(timeInterval, ecefFrame);

            // check
            assertEquals(eciFrame1.getX(), eciFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getY(), eciFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getZ(), eciFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getVx(), eciFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVy(), eciFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVz(), eciFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getCoordinateTransformation().getSourceType(),
                    eciFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(eciFrame1.getCoordinateTransformation().getDestinationType(),
                    eciFrame2.getCoordinateTransformation().getDestinationType());

            final Quaternion q1 = new Quaternion();
            eciFrame1.getCoordinateTransformation().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            eciFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(eciFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testConvert() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformation c = new CoordinateTransformation(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            final ECIFrame eciFrame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

            final ECItoECEFFrameConverter converter1 = new ECItoECEFFrameConverter();

            final ECEFFrame ecefFrame = new ECEFFrame();
            converter1.convert(timeInterval, eciFrame1, ecefFrame);

            // convert back to ECI
            final ECEFtoECIFrameConverter converter2 = new ECEFtoECIFrameConverter();

            final ECIFrame eciFrame2 = new ECIFrame();
            converter2.convert(timeInterval, ecefFrame, eciFrame2);

            // check
            assertEquals(eciFrame1.getX(), eciFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getY(), eciFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getZ(), eciFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getVx(), eciFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVy(), eciFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVz(), eciFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getCoordinateTransformation().getSourceType(),
                    eciFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(eciFrame1.getCoordinateTransformation().getDestinationType(),
                    eciFrame2.getCoordinateTransformation().getDestinationType());

            final Quaternion q1 = new Quaternion();
            eciFrame1.getCoordinateTransformation().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            eciFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(eciFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testConvertECIToECEFAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformation c = new CoordinateTransformation(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            final ECIFrame eciFrame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

            final ECEFFrame ecefFrame = ECItoECEFFrameConverter.convertECItoECEFAndReturnNew(
                    timeInterval, eciFrame1);

            // convert back to ECI
            final ECIFrame eciFrame2 = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                    timeInterval, ecefFrame);

            // check
            assertEquals(eciFrame1.getX(), eciFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getY(), eciFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getZ(), eciFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getVx(), eciFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVy(), eciFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVz(), eciFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getCoordinateTransformation().getSourceType(),
                    eciFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(eciFrame1.getCoordinateTransformation().getDestinationType(),
                    eciFrame2.getCoordinateTransformation().getDestinationType());

            final Quaternion q1 = new Quaternion();
            eciFrame1.getCoordinateTransformation().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            eciFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(eciFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testConvertECItoECEF() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final double vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final double vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Quaternion q = new Quaternion(roll, pitch, yaw);

            final Matrix m = q.asInhomogeneousMatrix();
            final CoordinateTransformation c = new CoordinateTransformation(
                    m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            final ECIFrame eciFrame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

            final ECEFFrame ecefFrame = new ECEFFrame();
            ECItoECEFFrameConverter.convertECItoECEF(timeInterval, eciFrame1, ecefFrame);

            // convert back to ECI
            final ECIFrame eciFrame2 = new ECIFrame();
            ECEFtoECIFrameConverter.convertECEFtoECI(timeInterval, ecefFrame, eciFrame2);

            // check
            assertEquals(eciFrame1.getX(), eciFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getY(), eciFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getZ(), eciFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getVx(), eciFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVy(), eciFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(eciFrame1.getVz(), eciFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(eciFrame1.getCoordinateTransformation().getSourceType(),
                    eciFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(eciFrame1.getCoordinateTransformation().getDestinationType(),
                    eciFrame2.getCoordinateTransformation().getDestinationType());

            final Quaternion q1 = new Quaternion();
            eciFrame1.getCoordinateTransformation().asRotation(q1);

            final Quaternion q2 = new Quaternion();
            eciFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(eciFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }
}
