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
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class NEDtoECEFFrameConverterTest {

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(NEDtoECEFFrameConverter.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                0.0);
        assertEquals(NEDtoECEFFrameConverter.EARTH_ECCENTRICITY, Constants.EARTH_ECCENTRICITY, 0.0);
    }

    @Test
    void testConstructor() {
        final var converter = new NEDtoECEFFrameConverter();

        // check
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, converter.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, converter.getDestinationType());
    }

    @Test
    void testConvertAndReturnNew() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

            final var converter1 = new NEDtoECEFFrameConverter();

            final var ecefFrame = converter1.convertAndReturnNew(nedFrame1);

            // convert back to NED
            final var converter2 = new ECEFtoNEDFrameConverter();

            final var nedFrame2 = converter2.convertAndReturnNew(ecefFrame);

            // check
            assertEquals(nedFrame1.getLatitude(), nedFrame2.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getLongitude(), nedFrame2.getLongitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getHeight(), nedFrame2.getHeight(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getVn(), nedFrame2.getVn(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVe(), nedFrame2.getVe(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVd(), nedFrame2.getVd(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getCoordinateTransformation().getSourceType(),
                    nedFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(nedFrame1.getCoordinateTransformation().getDestinationType(),
                    nedFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            nedFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            nedFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is the same either on ECEF or NED frame
            assertEquals(nedFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvert() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

            final var converter1 = new NEDtoECEFFrameConverter();

            final var ecefFrame = new ECEFFrame();
            converter1.convert(nedFrame1, ecefFrame);

            // convert back to NED
            final var converter2 = new ECEFtoNEDFrameConverter();

            final var nedFrame2 = new NEDFrame();
            converter2.convert(ecefFrame, nedFrame2);

            // check
            assertEquals(nedFrame1.getLatitude(), nedFrame2.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getLongitude(), nedFrame2.getLongitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getHeight(), nedFrame2.getHeight(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getVn(), nedFrame2.getVn(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVe(), nedFrame2.getVe(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVd(), nedFrame2.getVd(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getCoordinateTransformation().getSourceType(),
                    nedFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(nedFrame1.getCoordinateTransformation().getDestinationType(),
                    nedFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            nedFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            nedFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is the same either on ECEF or NED frame
            assertEquals(nedFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertNEDtoECEFAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // convert back to NED
            final var nedFrame2 = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(ecefFrame);

            // check
            assertEquals(nedFrame1.getLatitude(), nedFrame2.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getLongitude(), nedFrame2.getLongitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getHeight(), nedFrame2.getHeight(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getVn(), nedFrame2.getVn(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVe(), nedFrame2.getVe(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVd(), nedFrame2.getVd(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getCoordinateTransformation().getSourceType(),
                    nedFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(nedFrame1.getCoordinateTransformation().getDestinationType(),
                    nedFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            nedFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            nedFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is the same either on ECEF or NED frame
            assertEquals(nedFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertNEDtoECEF() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

            final var ecefFrame = new ECEFFrame();
            NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame1, ecefFrame);

            // convert back to NED
            final var nedFrame2 = new NEDFrame();
            ECEFtoNEDFrameConverter.convertECEFtoNED(ecefFrame, nedFrame2);

            // check
            assertEquals(nedFrame1.getLatitude(), nedFrame2.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getLongitude(), nedFrame2.getLongitude(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getHeight(), nedFrame2.getHeight(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getVn(), nedFrame2.getVn(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVe(), nedFrame2.getVe(), ABSOLUTE_ERROR);
            assertEquals(nedFrame1.getVd(), nedFrame2.getVd(), ABSOLUTE_ERROR);

            assertEquals(nedFrame1.getCoordinateTransformation().getSourceType(),
                    nedFrame2.getCoordinateTransformation().getSourceType());
            assertEquals(nedFrame1.getCoordinateTransformation().getDestinationType(),
                    nedFrame2.getCoordinateTransformation().getDestinationType());

            final var q1 = new Quaternion();
            nedFrame1.getCoordinateTransformation().asRotation(q1);

            final var q2 = new Quaternion();
            nedFrame2.getCoordinateTransformation().asRotation(q2);
            assertTrue(q1.equals(q2, ABSOLUTE_ERROR));

            // velocity norm is the same either on ECEF or NED frame
            assertEquals(nedFrame1.getVelocityNorm(), ecefFrame.getVelocityNorm(), ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }
}
