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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class NEDtoECEFPositionVelocityConverterTest {

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
        assertEquals(NEDtoECEFPositionVelocityConverter.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(NEDtoECEFPositionVelocityConverter.EARTH_ECCENTRICITY, Constants.EARTH_ECCENTRICITY, 0.0);
    }

    @Test
    void testConvert() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {
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

            final var nedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var nedPosition1 = nedFrame.getPosition();
            final var nedVelocity1 = nedFrame.getVelocity();

            // convert
            final var converter = new NEDtoECEFPositionVelocityConverter();
            final var ecefPosition1 = new ECEFPosition();
            final var ecefVelocity1 = new ECEFVelocity();
            converter.convert(latitude, longitude, height, vn, ve, vd, ecefPosition1, ecefVelocity1);

            final var ecefPosition2 = new ECEFPosition();
            final var ecefVelocity2 = new ECEFVelocity();
            converter.convert(nedPosition1, nedVelocity1, ecefPosition2, ecefVelocity2);

            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
            final var ecefPosition3 = ecefFrame.getECEFPosition();
            final var ecefVelocity3 = ecefFrame.getECEFVelocity();

            // check
            assertEquals(ecefPosition1, ecefPosition2);
            assertEquals(ecefVelocity1, ecefVelocity2);
            assertTrue(ecefPosition1.equals(ecefPosition3, ABSOLUTE_ERROR));
            assertTrue(ecefVelocity1.equals(ecefVelocity3, ABSOLUTE_ERROR));

            // convert back
            final var nedPosition2 = new NEDPosition();
            final var nedVelocity2 = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(ecefPosition1, ecefVelocity1, nedPosition2,
                    nedVelocity2);

            // check
            assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(nedVelocity1.equals(nedVelocity2, ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testConvertNEDtoECEF() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {
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

            final var nedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var nedPosition1 = nedFrame.getPosition();
            final var nedVelocity1 = nedFrame.getVelocity();

            // convert
            final var ecefPosition1 = new ECEFPosition();
            final var ecefVelocity1 = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(latitude, longitude, height, vn, ve, vd, ecefPosition1,
                    ecefVelocity1);

            final var ecefPosition2 = new ECEFPosition();
            final var ecefVelocity2 = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition1, nedVelocity1, ecefPosition2,
                    ecefVelocity2);

            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
            final var ecefPosition3 = ecefFrame.getECEFPosition();
            final var ecefVelocity3 = ecefFrame.getECEFVelocity();

            // check
            assertEquals(ecefPosition1, ecefPosition2);
            assertEquals(ecefVelocity1, ecefVelocity2);
            assertTrue(ecefPosition1.equals(ecefPosition3, ABSOLUTE_ERROR));
            assertTrue(ecefVelocity1.equals(ecefVelocity3, ABSOLUTE_ERROR));

            // convert back
            final var nedPosition2 = new NEDPosition();
            final var nedVelocity2 = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(ecefPosition1, ecefVelocity1, nedPosition2,
                    nedVelocity2);

            // check
            assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(nedVelocity1.equals(nedVelocity2, ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }
}
