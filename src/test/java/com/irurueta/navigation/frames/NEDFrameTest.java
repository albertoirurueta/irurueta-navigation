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
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class NEDFrameTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstants() {
        assertEquals(3, NEDFrame.NUM_VELOCITY_COORDINATES);
    }

    @Test
    void testConstructor() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        // test empty constructor
        var frame = new NEDFrame();

        // check
        assertEquals(0.0, frame.getLatitude(), 0.0);
        assertEquals(0.0, frame.getLongitude(), 0.0);
        assertEquals(0.0, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(0.0, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        CoordinateTransformation c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with coordinate transformation matrix
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        frame = new NEDFrame(c1);

        // check
        assertEquals(0.0, frame.getLatitude(), 0.0);
        assertEquals(0.0, frame.getLongitude(), 0.0);
        assertEquals(0.0, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(0.0, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        CoordinateTransformation c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position coordinates
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        frame = new NEDFrame(latitude, longitude, height);

        // check
        assertEquals(frame.getLatitude(), latitude, 0.0);
        assertEquals(frame.getLongitude(), longitude, 0.0);
        assertEquals(frame.getHeight(), height, 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());


        // test constructor with position coordinates
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        frame = new NEDFrame(latitude, longitude, heightDistance);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());


        // test constructor with position coordinates
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        frame = new NEDFrame(latitudeAngle, longitudeAngle, height);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position coordinates
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with NED position
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        frame = new NEDFrame(nedPosition);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinates
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinate
        frame = new NEDFrame(latitude, longitude, heightDistance, vn, ve, vd);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinate
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height, vn, ve, vd);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinates
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance, vn, ve, vd);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinates
        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        frame = new NEDFrame(latitude, longitude, height, speedN, speedE, speedD);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinates
        frame = new NEDFrame(latitude, longitude, heightDistance, speedN, speedE, speedD);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinates
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height, speedN, speedE, speedD);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinates
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance, speedN, speedE, speedD);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with NED position and NED velocity
        final var nedVelocity = new NEDVelocity(vn, ve, vd);
        frame = new NEDFrame(nedPosition, nedVelocity);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and coordinate transformation matrix
        frame = new NEDFrame(latitude, longitude, height, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitude, longitude, height, new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position and coordinate transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitudeAngle, longitudeAngle, height, new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME)));

        // test constructor with position and coordinate transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitudeAngle, longitudeAngle, heightDistance, new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME)));

        // test constructor with NED position and coordinate transformation matrix
        frame = new NEDFrame(nedPosition, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(nedPosition,
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position and velocity coordinates, and with coordinate transformation
        // matrix
        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitude, longitude, height, vn, ve, vd, new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME)));

        // test constructor with position, velocity coordinates and coordinate transformation matrix
        frame = new NEDFrame(latitude, longitude, heightDistance, vn, ve, vd, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitude, longitude, heightDistance, vn, ve, vd, new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME)));

        // test constructor with position, velocity coordinates and coordinate transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height, vn, ve, vd, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(frame.getVn(), vn, 0.0);
        assertEquals(frame.getVe(), ve, 0.0);
        assertEquals(frame.getVd(), vd, 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitudeAngle, longitudeAngle, height, vn, ve, vd, new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME)));

        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance, vn, ve, vd, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitudeAngle, longitudeAngle, heightDistance, vn, ve, vd, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitude, longitude, height, speedN, speedE, speedD, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitude, longitude, height, speedN, speedE, speedD, new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME)));

        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitude, longitude, heightDistance, speedN, speedE, speedD, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitude, longitude, heightDistance, speedN, speedE, speedD, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, height, speedN, speedE, speedD, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitudeAngle, longitudeAngle, height, speedN, speedE, speedD, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position, velocity coordinates and coordinates transformation matrix
        frame = new NEDFrame(latitudeAngle, longitudeAngle, heightDistance, speedN, speedE, speedD, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(
                latitudeAngle, longitudeAngle, height, speedN, speedE, speedD, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with NED position, NED velocity and coordinates transformation matrix
        frame = new NEDFrame(nedPosition, nedVelocity, c1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        assertEquals(latitude, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(longitude, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);
        assertEquals(height, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        assertEquals(vn, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(ve, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(vd, frame.getSpeedD().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new NEDFrame(nedPosition, nedVelocity,
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with another NED frame
        frame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c1);
        final var frame2 = new NEDFrame(frame);

        // check
        assertEquals(frame.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame.getHeight(), frame2.getHeight(), 0.0);

        assertEquals(frame.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame.getVd(), frame2.getVd(), 0.0);

        assertEquals(frame.getCoordinateTransformation(), frame2.getCoordinateTransformation());
    }

    @Test
    void testGetSetLatitude() {

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getLatitude(), 0.0);

        // set new value
        frame.setLatitude(latitude);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
    }

    @Test
    void testGetSetLongitude() {

        final var randomizer = new UniformRandomizer();
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getLongitude(), 0.0);

        // set new value
        frame.setLongitude(longitude);

        // check
        assertEquals(longitude, frame.getLongitude(), 0.0);
    }

    @Test
    void testGetSetHeight() {

        final var randomizer = new UniformRandomizer();
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getHeight(), 0.0);

        // set new value
        frame.setHeight(height);

        // check
        assertEquals(height, frame.getHeight(), 0.0);
    }

    @Test
    void testSetPosition1() {

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var frame = new NEDFrame();

        // check initial values
        assertEquals(0.0, frame.getLatitude(), 0.0);
        assertEquals(0.0, frame.getLongitude(), 0.0);
        assertEquals(0.0, frame.getHeight(), 0.0);

        // set new values
        frame.setPosition(latitude, longitude, height);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);
    }

    @Test
    void testGetSetLatitudeAngle() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getLatitudeAngle().getValue().doubleValue(), 0.0);

        // set new value
        final var latitudeAngle1 = new Angle(latitude, AngleUnit.RADIANS);
        frame.setLatitudeAngle(latitudeAngle1);

        // check
        final var latitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        frame.getLatitudeAngle(latitudeAngle2);
        final var latitudeAngle3 = frame.getLatitudeAngle();

        assertEquals(latitudeAngle1, latitudeAngle2);
        assertEquals(latitudeAngle1, latitudeAngle3);
    }

    @Test
    void testGetSetLongitudeAngle() {
        final var randomizer = new UniformRandomizer();
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getLongitudeAngle().getValue().doubleValue(), 0.0);

        // set new value
        final var longitudeAngle1 = new Angle(longitude, AngleUnit.RADIANS);
        frame.setLongitudeAngle(longitudeAngle1);

        // check
        final var longitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        frame.getLongitudeAngle(longitudeAngle2);
        final var longitudeAngle3 = frame.getLongitudeAngle();

        assertEquals(longitudeAngle1, longitudeAngle2);
        assertEquals(longitudeAngle1, longitudeAngle3);
    }

    @Test
    void testGetSetHeightDistance() {

        final var randomizer = new UniformRandomizer();
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getHeightDistance().getValue().doubleValue(), 0.0);

        // set new value
        final var heightDistance1 = new Distance(height, DistanceUnit.METER);
        frame.setHeightDistance(heightDistance1);

        // check
        final var heightDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        frame.getHeightDistance(heightDistance2);
        final var heightDistance3 = frame.getHeightDistance();

        assertEquals(heightDistance1, heightDistance2);
        assertEquals(heightDistance1, heightDistance3);
    }

    @Test
    void testSetPosition2() {

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var frame = new NEDFrame();

        // check initial values
        assertEquals(0.0, frame.getLatitude(), 0.0);
        assertEquals(0.0, frame.getLongitude(), 0.0);
        assertEquals(0.0, frame.getHeight(), 0.0);

        // set new values
        frame.setPosition(latitude, longitude, heightDistance);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);
    }

    @Test
    void testSetPosition3() {

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var frame = new NEDFrame();

        // check initial values
        assertEquals(0.0, frame.getLatitude(), 0.0);
        assertEquals(0.0, frame.getLongitude(), 0.0);
        assertEquals(0.0, frame.getHeight(), 0.0);

        // set new values
        frame.setPosition(latitudeAngle, longitudeAngle, height);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);
    }

    @Test
    void testSetPosition4() {

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var frame = new NEDFrame();

        // check initial values
        assertEquals(0.0, frame.getLatitude(), 0.0);
        assertEquals(0.0, frame.getLongitude(), 0.0);
        assertEquals(0.0, frame.getHeight(), 0.0);

        // set new values
        frame.setPosition(latitudeAngle, longitudeAngle, heightDistance);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);
    }

    @Test
    void testGetSetNEDPosition() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var frame = new NEDFrame();

        // check initial values
        assertEquals(0.0, frame.getLatitude(), 0.0);
        assertEquals(0.0, frame.getLongitude(), 0.0);
        assertEquals(0.0, frame.getHeight(), 0.0);

        // set new values
        final var position1 = new NEDPosition(latitude, longitude, height);
        frame.setPosition(position1);

        // check
        assertEquals(latitude, frame.getLatitude(), 0.0);
        assertEquals(longitude, frame.getLongitude(), 0.0);
        assertEquals(height, frame.getHeight(), 0.0);

        final var position2 = new NEDPosition();
        frame.getPosition(position2);
        final var position3 = frame.getPosition();

        assertEquals(latitude, position2.getLatitude(), 0.0);
        assertEquals(longitude, position2.getLongitude(), 0.0);
        assertEquals(height, position2.getHeight(), 0.0);

        assertEquals(position2, position3);
    }

    @Test
    void testGetSetVn() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getVn(), 0.0);

        // set new value
        frame.setVn(vn);

        // check
        assertEquals(vn, frame.getVn(), 0.0);
    }

    @Test
    void testGetSetVe() {

        final var randomizer = new UniformRandomizer();
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getVe(), 0.0);

        // set new value
        frame.setVe(ve);

        // check
        assertEquals(ve, frame.getVe(), 0.0);
    }

    @Test
    void testGetSetVd() {

        final var randomizer = new UniformRandomizer();
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getVd(), 0.0);

        // set new value
        frame.setVd(vd);

        // check
        assertEquals(vd, frame.getVd(), 0.0);
    }

    @Test
    void testSetVelocityCoordinates() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial values
        assertEquals(0.0, frame.getVn(), 0.0);
        assertEquals(0.0, frame.getVe(), 0.0);
        assertEquals(0.0, frame.getVd(), 0.0);

        // set new values
        frame.setVelocityCoordinates(vn, ve, vd);

        // check
        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);
    }

    @Test
    void testGetVelocityNorm() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var norm = Math.sqrt(Math.pow(vn, 2.0) + Math.pow(ve, 2.0) + Math.pow(vd, 2.0));

        final var frame = new NEDFrame();
        frame.setVelocityCoordinates(vn, ve, vd);

        assertEquals(norm, frame.getVelocityNorm(), ABSOLUTE_ERROR);

        final var normSpeed1 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getVelocityNormAsSpeed(normSpeed1);
        final var normSpeed2 = frame.getVelocityNormAsSpeed();

        assertEquals(norm, normSpeed1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(SpeedUnit.METERS_PER_SECOND, normSpeed1.getUnit());
        assertEquals(normSpeed1, normSpeed2);
    }

    @Test
    void testGetSetSpeedN() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);

        // set new value
        final var speedN1 = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedN(speedN1);

        // check
        final var speedN2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedN(speedN2);
        final var speedN3 = frame.getSpeedN();

        assertEquals(speedN1, speedN2);
        assertEquals(speedN1, speedN3);
    }

    @Test
    void testGetSetSpeedE() {

        final var randomizer = new UniformRandomizer();
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);

        // set new value
        final var speedE1 = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedE(speedE1);

        // check
        final var speedE2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedE(speedE2);
        final var speedE3 = frame.getSpeedE();

        assertEquals(speedE1, speedE2);
        assertEquals(speedE1, speedE3);
    }

    @Test
    void testGetSetSpeedD() {

        final var randomizer = new UniformRandomizer();
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial value
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        // set new value
        final var speedD1 = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedD(speedD1);

        // check
        final var speedD2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedD(speedD2);
        final var speedD3 = frame.getSpeedD();

        assertEquals(speedD1, speedD2);
        assertEquals(speedD1, speedD3);
    }

    @Test
    void testSetSpeedCoordinates() {

        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial values
        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        // set new values
        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        frame.setSpeedCoordinates(speedN, speedE, speedD);

        // check
        assertEquals(speedN, frame.getSpeedN());
        assertEquals(speedE, frame.getSpeedE());
        assertEquals(speedD, frame.getSpeedD());
    }

    @Test
    void testGetSetNEDVelocity() {
        final var randomizer = new UniformRandomizer();
        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new NEDFrame();

        // check initial values
        assertEquals(0.0, frame.getSpeedN().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedE().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedD().getValue().doubleValue(), 0.0);

        final var velocity1 = new NEDVelocity(vn, ve, vd);
        frame.setVelocity(velocity1);

        // check
        assertEquals(vn, frame.getVn(), 0.0);
        assertEquals(ve, frame.getVe(), 0.0);
        assertEquals(vd, frame.getVd(), 0.0);

        final var velocity2 = new NEDVelocity();
        frame.getVelocity(velocity2);
        final var velocity3 = frame.getVelocity();

        assertEquals(vn, velocity2.getVn(), 0.0);
        assertEquals(ve, velocity2.getVe(), 0.0);
        assertEquals(vd, velocity2.getVd(), 0.0);

        assertEquals(velocity2, velocity3);
    }

    @Test
    void testGetSetCoordinateTransformation() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final var frame = new NEDFrame();

        // check initial value
        var c1 = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c1.getSourceType());
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c1.getDestinationType());
        assertEquals(c1.getMatrix(), Matrix.identity(3, 3));

        // set new value
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c2 = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        frame.setCoordinateTransformation(c2);

        // check
        assertEquals(frame.getCoordinateTransformation(), c2);
        final var c3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        frame.getCoordinateTransformation(c3);
        assertEquals(c2, c3);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> frame.setCoordinateTransformation(
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testGetSetCoordinateTransformationMatrix() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m1 = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m1, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var frame = new NEDFrame(c);

        // check
        assertEquals(frame.getCoordinateTransformationMatrix(), m1);
        final var m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m1, m2);
    }

    @Test
    void testGetSetCoordinateTransformationMatrix2() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m1 = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var frame = new NEDFrame(c);

        // check default value
        assertEquals(Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS),
                frame.getCoordinateTransformationMatrix());
        final var m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS), m2);

        // set nw value
        frame.setCoordinateTransformationMatrix(m1, THRESHOLD);

        // check
        assertEquals(m1, frame.getCoordinateTransformationMatrix());
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m1, m2);
    }

    @Test
    void testGetSetCoordinateTransformationMatrix3() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m1 = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var frame = new NEDFrame(c);

        // check default value
        assertEquals(Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS),
                frame.getCoordinateTransformationMatrix());
        final var m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS), m2);

        // set nw value
        frame.setCoordinateTransformationMatrix(m1);

        // check
        assertEquals(m1, frame.getCoordinateTransformationMatrix());
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m1, m2);
    }

    @Test
    void testGetSetCoordinateTransformationRotation() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var frame = new NEDFrame(c);

        // check default value
        assertEquals(new Quaternion(), frame.getCoordinateTransformationRotation());
        final var q2 = new Quaternion();
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
    void testIsValidCoordinateTransformation() {
        final var c1 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var c2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final var c3 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        assertTrue(NEDFrame.isValidCoordinateTransformation(c1));
        assertFalse(NEDFrame.isValidCoordinateTransformation(c2));
        assertFalse(NEDFrame.isValidCoordinateTransformation(c3));
    }

    @Test
    void testCopyTo() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

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

        final var frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var frame2 = new NEDFrame();
        frame1.copyTo(frame2);

        // check
        assertEquals(frame1.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame1.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame1.getHeight(), frame2.getHeight(), 0.0);
        assertEquals(frame1.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame1.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame1.getVd(), frame2.getVd(), 0.0);
        assertEquals(frame1.getCoordinateTransformation(), frame2.getCoordinateTransformation());
    }

    @Test
    void testCopyFrom() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

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

        final var frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var frame2 = new NEDFrame();
        frame2.copyFrom(frame1);

        // check
        assertEquals(frame1.getLatitude(), frame2.getLatitude(), 0.0);
        assertEquals(frame1.getLongitude(), frame2.getLongitude(), 0.0);
        assertEquals(frame1.getHeight(), frame2.getHeight(), 0.0);
        assertEquals(frame1.getVn(), frame2.getVn(), 0.0);
        assertEquals(frame1.getVe(), frame2.getVe(), 0.0);
        assertEquals(frame1.getVd(), frame2.getVd(), 0.0);
        assertEquals(frame1.getCoordinateTransformation(), frame2.getCoordinateTransformation());
    }

    @Test
    void testHashCode() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

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

        final var frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var frame3 = new NEDFrame();

        assertEquals(frame1.hashCode(), frame2.hashCode());
        assertNotEquals(frame1.hashCode(), frame3.hashCode());
    }

    @Test
    void testEquals() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

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

        final var frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var frame3 = new NEDFrame();

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
        assertNotEquals(new Object(), frame1);
    }

    @Test
    void testEqualsWithThreshold() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

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

        final var frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var frame2 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var frame3 = new NEDFrame();

        assertTrue(frame1.equals(frame1, THRESHOLD));
        assertTrue(frame1.equals(frame2, THRESHOLD));
        assertFalse(frame1.equals(frame3, THRESHOLD));
        assertFalse(frame1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            CloneNotSupportedException {

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

        final var frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

        final var frame2 = frame1.clone();

        assertEquals(frame1, frame2);
    }

    @Test
    void testSerializeDeserialize() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame1 = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

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
        final var bytes = SerializationHelper.serialize(frame1);
        final var frame2 = SerializationHelper.<NEDFrame>deserialize(bytes);

        // check
        assertNotSame(frame1, frame2);
        assertEquals(frame1, frame2);
    }
}
