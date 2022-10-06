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
package com.irurueta.navigation.frames;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class NEDFrameTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstants() {

        assertEquals(NEDFrame.NUM_VELOCITY_COORDINATES, 3);
    }

    @Test
    public void testConstructor() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        // test empty constructor
        NEDFrame frame = new NEDFrame();

        // check
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), 0.0, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        CoordinateTransformation c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with coordinate transformation matrix
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation c1 = new CoordinateTransformation(
                m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        frame = new NEDFrame(c1);

        // check
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), 0.0, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        CoordinateTransformation c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(new CoordinateTransformation(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position coordinates
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        frame = new NEDFrame(latitude, longitude, height);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position coordinates
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        frame = new NEDFrame(latitude, longitude, heightDistance);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position coordinates
        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        frame = new NEDFrame(latitudeAngle, longitudeAngle, height);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position coordinates
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with NED position
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        frame = new NEDFrame(nedPosition);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinate
        frame = new NEDFrame(latitude, longitude, heightDistance, vn, ve, vd);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinate
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height, vn, ve, vd);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance, vn, ve, vd);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        frame = new NEDFrame(latitude, longitude, height, speedN, speedE, speedD);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        frame = new NEDFrame(latitude, longitude, heightDistance, speedN, speedE, speedD);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height,
                speedN, speedE, speedD);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and velocity coordinates
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance,
                speedN, speedE, speedD);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with NED position and NED velocity
        final NEDVelocity nedVelocity = new NEDVelocity(vn, ve, vd);
        frame = new NEDFrame(nedPosition, nedVelocity);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(c.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c.getMatrix(), Matrix.identity(3, 3));


        // test constructor with position and coordinate transformation matrix
        frame = new NEDFrame(latitude, longitude, height, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitude, longitude, height,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position and coordinate transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitudeAngle, longitudeAngle, height,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position and coordinate transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with NED position and coordinate transformation matrix
        frame = new NEDFrame(nedPosition, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(nedPosition,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position and velocity coordinates, and with coordinate transformation matrix
        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitude, longitude, height, vn, ve, vd,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position, velocity coordinates and coordinate transformation matrix
        frame = new NEDFrame(latitude, longitude, heightDistance, vn, ve, vd, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitude, longitude, heightDistance, vn, ve, vd,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position, velocity coordinates and coordinate transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height, vn, ve, vd, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitudeAngle, longitudeAngle, height, vn, ve, vd,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance, vn, ve, vd, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance,
                    vn, ve, vd,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitude, longitude, height, speedN, speedE, speedD, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitude, longitude, height, speedN, speedE, speedD,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitude, longitude, heightDistance,
                speedN, speedE, speedD, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitude, longitude, heightDistance,
                    speedN, speedE, speedD,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height,
                speedN, speedE, speedD, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitudeAngle, longitudeAngle, height,
                    speedN, speedE, speedD,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance,
                speedN, speedE, speedD, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(latitudeAngle, longitudeAngle, height,
                    speedN, speedE, speedD,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with NED position, NED velocity and coordinates transformation matrix
        frame = new NEDFrame(nedPosition, nedVelocity, c1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), height, 0.0);

        assertEquals(frame.getSpeedN().getValue().doubleValue(), vn, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), ve, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), vd, 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        frame = null;
        try {
            frame = new NEDFrame(nedPosition, nedVelocity,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
        assertNull(frame);


        // test constructor with another NED frame
        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c1);
        final NEDFrame frame2 = new NEDFrame(frame);

        // check
        assertEquals(frame.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame.getHeight(), frame2.getHeight(), 0.0);

        assertEquals(frame.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame.getVd(), frame2.getVd(), 0.0);

        assertEquals(frame.getCoordinateTransformation(),
                frame2.getCoordinateTransformation());
    }

    @Test
    public void testGetSetLatitude() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getLatitude(), 0.0, 0.0);

        // set new value
        frame.setLatitude(latitude);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
    }

    @Test
    public void testGetSetLongitude() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getLongitude(), 0.0, 0.0);

        // set new value
        frame.setLongitude(longitude);

        // check
        assertEquals(frame.getLongitude(), longitude, 0.0);
    }

    @Test
    public void testGetSetHeight() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getHeight(), 0.0, 0.0);

        // set new value
        frame.setHeight(height);

        // check
        assertEquals(frame.getHeight(), height, 0.0);
    }

    @Test
    public void testSetPosition1() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        // set new values
        frame.setPosition(latitude, longitude, height);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);
    }

    @Test
    public void testGetSetLatitudeAngle() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getLatitudeAngle().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Angle latitudeAngle1 = new Angle(latitude, AngleUnit.RADIANS);
        frame.setLatitudeAngle(latitudeAngle1);

        // check
        final Angle latitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        frame.getLatitudeAngle(latitudeAngle2);
        final Angle latitudeAngle3 = frame.getLatitudeAngle();

        assertEquals(latitudeAngle1, latitudeAngle2);
        assertEquals(latitudeAngle1, latitudeAngle3);
    }

    @Test
    public void testGetSetLongitudeAngle() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getLongitudeAngle().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Angle longitudeAngle1 = new Angle(longitude, AngleUnit.RADIANS);
        frame.setLongitudeAngle(longitudeAngle1);

        // check
        final Angle longitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        frame.getLongitudeAngle(longitudeAngle2);
        final Angle longitudeAngle3 = frame.getLongitudeAngle();

        assertEquals(longitudeAngle1, longitudeAngle2);
        assertEquals(longitudeAngle1, longitudeAngle3);
    }

    @Test
    public void testGetSetHeightDistance() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getHeightDistance().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Distance heightDistance1 = new Distance(height, DistanceUnit.METER);
        frame.setHeightDistance(heightDistance1);

        // check
        final Distance heightDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        frame.getHeightDistance(heightDistance2);
        final Distance heightDistance3 = frame.getHeightDistance();

        assertEquals(heightDistance1, heightDistance2);
        assertEquals(heightDistance1, heightDistance3);
    }

    @Test
    public void testSetPosition2() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        // set new values
        frame.setPosition(latitude, longitude, heightDistance);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);
    }

    @Test
    public void testSetPosition3() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        // set new values
        frame.setPosition(latitudeAngle, longitudeAngle, height);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);
    }

    @Test
    public void testSetPosition4() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        // set new values
        frame.setPosition(latitudeAngle, longitudeAngle, heightDistance);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);
    }

    @Test
    public void testGetSetNEDPosition() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getLatitude(), 0.0, 0.0);
        assertEquals(frame.getLongitude(), 0.0, 0.0);
        assertEquals(frame.getHeight(), 0.0, 0.0);

        // set new values
        final NEDPosition position1 = new NEDPosition(latitude, longitude, height);
        frame.setPosition(position1);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        final NEDPosition position2 = new NEDPosition();
        frame.getPosition(position2);
        final NEDPosition position3 = frame.getPosition();

        assertEquals(position2.getLatitude(), latitude, 0.0);
        assertEquals(position2.getLongitude(), longitude, 0.0);
        assertEquals(position2.getHeight(), height, 0.0);

        assertEquals(position2, position3);
    }

    @Test
    public void testGetSetVn() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getVn(), 0.0, 0.0);

        // set new value
        frame.setVn(vn);

        // check
        assertEquals(frame.getVn(), vn, 0.0);
    }

    @Test
    public void testGetSetVe() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getVe(), 0.0, 0.0);

        // set new value
        frame.setVe(ve);

        // check
        assertEquals(frame.getVe(), ve, 0.0);
    }

    @Test
    public void testGetSetVd() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getVd(), 0.0, 0.0);

        // set new value
        frame.setVd(vd);

        // check
        assertEquals(frame.getVd(), vd, 0.0);
    }

    @Test
    public void testSetVelocityCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getVn(), 0.0, 0.0);
        assertEquals(frame.getVe(), 0.0, 0.0);
        assertEquals(frame.getVd(), 0.0, 0.0);

        // set new values
        frame.setVelocityCoordinates(vn, ve, vd);

        // check
        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);
    }

    @Test
    public void testGetVelocityNorm() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double norm = Math.sqrt(Math.pow(vn, 2.0) + Math.pow(ve, 2.0)
                + Math.pow(vd, 2.0));

        final NEDFrame frame = new NEDFrame();
        frame.setVelocityCoordinates(vn, ve, vd);

        assertEquals(frame.getVelocityNorm(), norm, ABSOLUTE_ERROR);

        final Speed normSpeed1 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getVelocityNormAsSpeed(normSpeed1);
        final Speed normSpeed2 = frame.getVelocityNormAsSpeed();

        assertEquals(normSpeed1.getValue().doubleValue(), norm, ABSOLUTE_ERROR);
        assertEquals(normSpeed1.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(normSpeed1, normSpeed2);
    }

    @Test
    public void testGetSetSpeedN() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Speed speedN1 = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedN(speedN1);

        // check
        final Speed speedN2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedN(speedN2);
        final Speed speedN3 = frame.getSpeedN();

        assertEquals(speedN1, speedN2);
        assertEquals(speedN1, speedN3);
    }

    @Test
    public void testGetSetSpeedE() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Speed speedE1 = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedE(speedE1);

        // check
        final Speed speedE2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedE(speedE2);
        final Speed speedE3 = frame.getSpeedE();

        assertEquals(speedE1, speedE2);
        assertEquals(speedE1, speedE3);
    }

    @Test
    public void testGetSetSpeedD() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial value
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        // set new value
        final Speed speedD1 = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedD(speedD1);

        // check
        final Speed speedD2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedD(speedD2);
        final Speed speedD3 = frame.getSpeedD();

        assertEquals(speedD1, speedD2);
        assertEquals(speedD1, speedD3);
    }

    @Test
    public void testSetSpeedCoordinates() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        // set new values
        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        frame.setSpeedCoordinates(speedN, speedE, speedD);

        // check
        assertEquals(frame.getSpeedN(), speedN);
        assertEquals(frame.getSpeedE(), speedE);
        assertEquals(frame.getSpeedD(), speedD);
    }

    @Test
    public void testGetSetNEDVelocity() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame = new NEDFrame();

        // check initial values
        assertEquals(frame.getSpeedN().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedE().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(frame.getSpeedD().getValue().doubleValue(), 0.0, 0.0);

        final NEDVelocity velocity1 = new NEDVelocity(vn, ve, vd);
        frame.setVelocity(velocity1);

        // check
        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        final NEDVelocity velocity2 = new NEDVelocity();
        frame.getVelocity(velocity2);
        final NEDVelocity velocity3 = frame.getVelocity();

        assertEquals(velocity2.getVn(), vn, 0.0);
        assertEquals(velocity2.getVe(), ve, 0.0);
        assertEquals(velocity2.getVd(), vd, 0.0);

        assertEquals(velocity2, velocity3);
    }

    @Test
    public void testGetSetCoordinateTransformation() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final NEDFrame frame = new NEDFrame();

        // check initial value
        CoordinateTransformation c1 = frame.getCoordinateTransformation();
        assertEquals(c1.getSourceType(), FrameType.BODY_FRAME);
        assertEquals(c1.getDestinationType(), FrameType.LOCAL_NAVIGATION_FRAME);
        assertEquals(c1.getMatrix(), Matrix.identity(3, 3));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation c2 = new CoordinateTransformation(
                m, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        frame.setCoordinateTransformation(c2);

        // check
        assertEquals(frame.getCoordinateTransformation(), c2);
        final CoordinateTransformation c3 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        frame.getCoordinateTransformation(c3);
        assertEquals(c2, c3);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            frame.setCoordinateTransformation(new CoordinateTransformation(
                    FrameType.BODY_FRAME, FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testGetSetCoordinateTransformationMatrix() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m1 = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(
                m1, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame = new NEDFrame(c);

        // check
        assertEquals(frame.getCoordinateTransformationMatrix(), m1);
        final Matrix m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m2, m1);
    }

    @Test
    public void testGetSetCoordinateTransformationMatrix2() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m1 = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame = new NEDFrame(c);

        // check default value
        assertEquals(frame.getCoordinateTransformationMatrix(),
                Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS));
        final Matrix m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m2, Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS));

        // set nw value
        frame.setCoordinateTransformationMatrix(m1, THRESHOLD);

        // check
        assertEquals(m1, frame.getCoordinateTransformationMatrix());
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m1, m2);
    }

    @Test
    public void testGetSetCoordinateTransformationMatrix3() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m1 = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame = new NEDFrame(c);

        // check default value
        assertEquals(frame.getCoordinateTransformationMatrix(),
                Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS));
        final Matrix m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m2, Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS));

        // set nw value
        frame.setCoordinateTransformationMatrix(m1);

        // check
        assertEquals(m1, frame.getCoordinateTransformationMatrix());
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m1, m2);
    }

    @Test
    public void testGetSetCoordinateTransformationRotation() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final CoordinateTransformation c = new CoordinateTransformation(
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame = new NEDFrame(c);

        // check default value
        assertEquals(new Quaternion(), frame.getCoordinateTransformationRotation());
        final Quaternion q2 = new Quaternion();
        frame.getCoordinateTransformationRotation(q2);
        assertEquals(new Quaternion(), q2);

        // set new value
        frame.setCoordinateTransformationRotation(q);

        // check
        assertEquals(q, frame.getCoordinateTransformationRotation());
        frame.getCoordinateTransformationRotation(q2);
        assertEquals(q, q2);
    }

    @Test
    public void testIsValidCoordinateTransformation() {
        final CoordinateTransformation c1 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final CoordinateTransformation c2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final CoordinateTransformation c3 = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        assertTrue(NEDFrame.isValidCoordinateTransformation(c1));
        assertFalse(NEDFrame.isValidCoordinateTransformation(c2));
        assertFalse(NEDFrame.isValidCoordinateTransformation(c3));
    }

    @Test
    public void testCopyTo() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame();
        frame1.copyTo(frame2);

        // check
        assertEquals(frame1.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame1.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame1.getHeight(), frame2.getHeight(), 0.0);
        assertEquals(frame1.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame1.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame1.getVd(), frame2.getVd(), 0.0);
        assertEquals(frame1.getCoordinateTransformation(),
                frame2.getCoordinateTransformation());
    }

    @Test
    public void testCopyFrom() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame();
        frame2.copyFrom(frame1);

        // check
        assertEquals(frame1.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame1.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame1.getHeight(), frame2.getHeight(), 0.0);
        assertEquals(frame1.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame1.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame1.getVd(), frame2.getVd(), 0.0);
        assertEquals(frame1.getCoordinateTransformation(),
                frame2.getCoordinateTransformation());
    }

    @Test
    public void testHashCode() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame3 = new NEDFrame();

        assertEquals(frame1.hashCode(), frame2.hashCode());
        assertNotEquals(frame1.hashCode(), frame3.hashCode());
    }

    @Test
    public void testEquals() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame3 = new NEDFrame();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(frame1.equals((Object) frame1));
        //noinspection EqualsWithItself
        assertTrue(frame1.equals(frame1));
        assertTrue(frame1.equals(frame2));
        assertFalse(frame1.equals(frame3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(frame1.equals((Object) null));
        assertFalse(frame1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(frame1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final NEDFrame frame3 = new NEDFrame();

        assertTrue(frame1.equals(frame1, THRESHOLD));
        assertTrue(frame1.equals(frame2, THRESHOLD));
        assertFalse(frame1.equals(frame3, THRESHOLD));
        assertFalse(frame1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException,
            CloneNotSupportedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

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
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

        final Object frame2 = frame1.clone();

        assertEquals(frame1, frame2);
    }

    @Test
    public void testSerializeDeserialize() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toDegrees(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(
                m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final NEDFrame frame1 = new NEDFrame(
                latitude, longitude, height, vn, ve, vd, c);

        // check
        assertEquals(latitude, frame1.getLatitude(), 0.0);
        assertEquals(longitude, frame1.getLongitude(), 0.0);
        assertEquals(height, frame1.getHeight(), 0.0);
        assertEquals(vn, frame1.getVn(), 0.0);
        assertEquals(ve, frame1.getVe(), 0.0);
        assertEquals(vd, frame1.getVd(), 0.0);
        assertNotSame(c, frame1.getCoordinateTransformation());
        assertEquals(c, frame1.getCoordinateTransformation());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(frame1);
        final NEDFrame frame2 = SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(frame1, frame2);
        assertEquals(frame1, frame2);
    }
}
