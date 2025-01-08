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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ECEFtoECIFrameConverterTest {

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_Z_VALUE = -50.0;
    private static final double MAX_Z_VALUE = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(ECEFtoECIFrameConverter.EARTH_ROTATION_RATE, Constants.EARTH_ROTATION_RATE, 0.0);
    }

    @Test
    void testConstructor() {
        final var converter = new ECEFtoECIFrameConverter();

        // check
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, converter.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, converter.getDestinationType());
    }

    @Test
    void testConvertAndReturnNewWithSecondsTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final var converter1 = new ECEFtoECIFrameConverter();

            final var eciFrame = converter1.convertAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame1);

            // convert back to ECEF
            final var converter2 = new ECItoECEFFrameConverter();

            final var ecefFrame2 = converter2.convertAndReturnNew(TIME_INTERVAL_SECONDS, eciFrame);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformation().getSourceType(),
                    ecefFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformation().getDestinationType(),
                    ecefFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(ecefFrame1.getVelocityNorm(), eciFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertWithSecondsTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final var converter1 = new ECEFtoECIFrameConverter();

            final var eciFrame = new ECIFrame();
            converter1.convert(TIME_INTERVAL_SECONDS, ecefFrame1, eciFrame);

            // convert back to ECEF
            final var converter2 = new ECItoECEFFrameConverter();

            final var ecefFrame2 = new ECEFFrame();
            converter2.convert(TIME_INTERVAL_SECONDS, eciFrame, ecefFrame2);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformation().getSourceType(),
                    ecefFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformation().getDestinationType(),
                    ecefFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(ecefFrame1.getVelocityNorm(), eciFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertECEFtoECIAndReturnNewWithSecondsTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final var eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame1);

            // convert back to ECEF
            final var ecefFrame2 = ECItoECEFFrameConverter.convertECItoECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                    eciFrame);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformation().getSourceType(),
                    ecefFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformation().getDestinationType(),
                    ecefFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(ecefFrame1.getVelocityNorm(), eciFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertECEFtoECIWithSecondsTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final var eciFrame = new ECIFrame();
            ECEFtoECIFrameConverter.convertECEFtoECI(TIME_INTERVAL_SECONDS, ecefFrame1, eciFrame);

            // convert back to ECEF
            final var ecefFrame2 = new ECEFFrame();
            ECItoECEFFrameConverter.convertECItoECEF(TIME_INTERVAL_SECONDS, eciFrame, ecefFrame2);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformation().getSourceType(),
                    ecefFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformation().getDestinationType(),
                    ecefFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(ecefFrame1.getVelocityNorm(), eciFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertAndReturnNew() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            RotationException {

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final var converter1 = new ECEFtoECIFrameConverter();

            final var eciFrame = converter1.convertAndReturnNew(timeInterval, ecefFrame1);

            // convert back to ECEF
            final var converter2 = new ECItoECEFFrameConverter();

            final var ecefFrame2 = converter2.convertAndReturnNew(timeInterval, eciFrame);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformation().getSourceType(),
                    ecefFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformation().getDestinationType(),
                    ecefFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(ecefFrame1.getVelocityNorm(), eciFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvert() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            RotationException {

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final var converter1 = new ECEFtoECIFrameConverter();

            final var eciFrame = new ECIFrame();
            converter1.convert(timeInterval, ecefFrame1, eciFrame);

            // convert back to ECEF
            final var converter2 = new ECItoECEFFrameConverter();

            final var ecefFrame2 = new ECEFFrame();
            converter2.convert(timeInterval, eciFrame, ecefFrame2);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformation().getSourceType(),
                    ecefFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformation().getDestinationType(),
                    ecefFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(ecefFrame1.getVelocityNorm(), eciFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertECEFtoECIAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final var eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(timeInterval, ecefFrame1);

            // convert back to ECEF
            final var ecefFrame2 = ECItoECEFFrameConverter.convertECItoECEFAndReturnNew(timeInterval, eciFrame);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformation().getSourceType(),
                    ecefFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformation().getDestinationType(),
                    ecefFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(ecefFrame1.getVelocityNorm(), eciFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertECEFtoECI() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            RotationException {

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final var z = randomizer.nextDouble(MIN_Z_VALUE, MAX_Z_VALUE);

            final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var ecefFrame1 = new ECEFFrame(x, y, z, vx, vy, vz, c);

            final var eciFrame = new ECIFrame();
            ECEFtoECIFrameConverter.convertECEFtoECI(timeInterval, ecefFrame1, eciFrame);

            // convert back to ECEF
            final var ecefFrame2 = new ECEFFrame();
            ECItoECEFFrameConverter.convertECItoECEF(timeInterval, eciFrame, ecefFrame2);

            // check
            assertEquals(ecefFrame1.getX(), ecefFrame2.getX(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getY(), ecefFrame2.getY(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getZ(), ecefFrame2.getZ(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getVx(), ecefFrame2.getVx(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVy(), ecefFrame2.getVy(), ABSOLUTE_ERROR);
            assertEquals(ecefFrame1.getVz(), ecefFrame2.getVz(), ABSOLUTE_ERROR);

            assertEquals(ecefFrame1.getCoordinateTransformation().getSourceType(),
                    ecefFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(ecefFrame1.getCoordinateTransformation().getDestinationType(),
                    ecefFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            ecefFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            ecefFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is not the same on ECEF and ECI because ECI takes into account
            // Earth rotation
            assertNotEquals(ecefFrame1.getVelocityNorm(), eciFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }
}
