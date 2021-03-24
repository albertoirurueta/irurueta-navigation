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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ECEFtoNEDPositionVelocityConverterTest {

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final int TIMES = 100;

    @Test
    public void testConstants() {

        assertEquals(ECEFtoNEDPositionVelocityConverter.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(ECEFtoNEDPositionVelocityConverter.EARTH_ECCENTRICITY,
                Constants.EARTH_ECCENTRICITY, 0.0);
    }

    @Test
    public void testConvert() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

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
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final ECEFFrame ecefFrame = new ECEFFrame(x, y, z, vx, vy, vz, c);
            final ECEFPosition ecefPosition1 = ecefFrame.getECEFPosition();
            final ECEFVelocity ecefVelocity1 = ecefFrame.getECEFVelocity();

            // convert
            final ECEFtoNEDPositionVelocityConverter converter =
                    new ECEFtoNEDPositionVelocityConverter();
            final NEDPosition nedPosition1 = new NEDPosition();
            final NEDVelocity nedVelocity1 = new NEDVelocity();
            converter.convert(x, y, z, vx, vy, vz, nedPosition1, nedVelocity1);

            final NEDPosition nedPosition2 = new NEDPosition();
            final NEDVelocity nedVelocity2 = new NEDVelocity();
            converter.convert(ecefPosition1, ecefVelocity1, nedPosition2, nedVelocity2);

            final NEDFrame nedFrame = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(ecefFrame);
            final NEDPosition nedPosition3 = nedFrame.getPosition();
            final NEDVelocity nedVelocity3 = nedFrame.getVelocity();

            // check
            assertEquals(nedPosition1, nedPosition2);
            assertEquals(nedVelocity1, nedVelocity2);
            assertTrue(nedPosition1.equals(nedPosition3, ABSOLUTE_ERROR));
            assertTrue(nedVelocity1.equals(nedVelocity3, ABSOLUTE_ERROR));

            // convert back
            final ECEFPosition ecefPosition2 = new ECEFPosition();
            final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                    nedPosition1, nedVelocity1,
                    ecefPosition2, ecefVelocity2);

            // check
            assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));
            assertTrue(ecefVelocity1.equals(ecefVelocity2, ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testConvertECEFtoNED() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
            final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

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
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final ECEFFrame ecefFrame = new ECEFFrame(x, y, z, vx, vy, vz, c);
            final ECEFPosition ecefPosition1 = ecefFrame.getECEFPosition();
            final ECEFVelocity ecefVelocity1 = ecefFrame.getECEFVelocity();

            // convert
            final NEDPosition nedPosition1 = new NEDPosition();
            final NEDVelocity nedVelocity1 = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(x, y, z, vx, vy, vz,
                    nedPosition1, nedVelocity1);

            final NEDPosition nedPosition2 = new NEDPosition();
            final NEDVelocity nedVelocity2 = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(ecefPosition1,
                    ecefVelocity1, nedPosition2, nedVelocity2);

            final NEDFrame nedFrame = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(ecefFrame);
            final NEDPosition nedPosition3 = nedFrame.getPosition();
            final NEDVelocity nedVelocity3 = nedFrame.getVelocity();

            // check
            assertEquals(nedPosition1, nedPosition2);
            assertEquals(nedVelocity1, nedVelocity2);
            assertTrue(nedPosition1.equals(nedPosition3, ABSOLUTE_ERROR));
            assertTrue(nedVelocity1.equals(nedVelocity3, ABSOLUTE_ERROR));

            // convert back
            final ECEFPosition ecefPosition2 = new ECEFPosition();
            final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                    nedPosition1, nedVelocity1,
                    ecefPosition2, ecefVelocity2);

            // check
            assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));
            assertTrue(ecefVelocity1.equals(ecefVelocity2, ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }
}
