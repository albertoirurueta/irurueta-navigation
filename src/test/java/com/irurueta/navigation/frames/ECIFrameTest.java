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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class ECIFrameTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstants() {

        assertEquals(3, ECIFrame.NUM_POSITION_COORDINATES);
        assertEquals(3, ECIFrame.NUM_VELOCITY_COORDINATES);
    }

    @Test
    void testConstructor() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        var frame = new ECIFrame();

        // check
        assertEquals(0.0, frame.getX(), 0.0);
        assertEquals(0.0, frame.getY(), 0.0);
        assertEquals(0.0, frame.getZ(), 0.0);

        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVy(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        assertEquals(0.0, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        var c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with coordinate transformation matrix
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c1 = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        frame = new ECIFrame(c1);

        // check
        assertEquals(0.0, frame.getX(), 0.0);
        assertEquals(0.0, frame.getY(), 0.0);
        assertEquals(0.0, frame.getZ(), 0.0);

        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVy(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        assertEquals(0.0, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        CoordinateTransformation c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with cartesian position coordinates
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        frame = new ECIFrame(x, y, z);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVy(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());


        // test constructor with position
        final var position = new InhomogeneousPoint3D(x, y, z);

        frame = new ECIFrame(position);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVy(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position coordinates
        final var positionX = new Distance(x, DistanceUnit.METER);
        final var positionY = new Distance(y, DistanceUnit.METER);
        final var positionZ = new Distance(z, DistanceUnit.METER);

        frame = new ECIFrame(positionX, positionY, positionZ);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());


        // test constructor with cartesian position and velocity coordinates
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        frame = new ECIFrame(x, y, z, vx, vy, vz);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with position and velocity coordinates
        frame = new ECIFrame(position, vx, vy, vz);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());


        // test constructor with position and speed coordinates
        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        frame = new ECIFrame(position, speedX, speedY, speedZ);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());


        // test constructor with cartesian position coordinates and speed coordinates
        frame = new ECIFrame(x, y, z, speedX, speedY, speedZ);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());


        // test constructor with position and velocity coordinates
        frame = new ECIFrame(positionX, positionY, positionZ, vx, vy, vz);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());


        // test constructor with position and speed coordinates
        frame = new ECIFrame(positionX, positionY, positionZ, speedX, speedY, speedZ);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        assertNotNull(frame.getCoordinateTransformation());

        c = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c.getMatrix());

        // test constructor with cartesian position coordinates and coordinate transformation matrix
        frame = new ECIFrame(x, y, z, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVy(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(x, y, z,
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position and coordinate transformation matrix
        frame = new ECIFrame(position, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVy(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(position,
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position coordinates and coordinate transformation matrix
        frame = new ECIFrame(positionX, positionY, positionZ, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVy(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(
                positionX, positionY, positionZ, new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME)));

        // test constructor with cartesian position and velocity coordinates, and with coordinate
        // transformation matrix
        frame = new ECIFrame(x, y, z, vx, vy, vz, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(x, y, z, vx, vy, vz,
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position, velocity coordinates, and coordinate transformation matrix
        frame = new ECIFrame(position, vx, vy, vz, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(position, vx, vy, vz,
                new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position, speed coordinates and coordinate transformation matrix
        frame = new ECIFrame(position, speedX, speedY, speedZ, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(position,
                speedX, speedY, speedZ, new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with cartesian coordinates, speed coordinates and coordinate transformation
        // matrix
        frame = new ECIFrame(x, y, z, speedX, speedY, speedZ, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(x, y, z,
                speedX, speedY, speedZ, new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with position coordinates, velocity coordinates and coordinates transformation
        // matrix
        frame = new ECIFrame(positionX, positionY, positionZ, vx, vy, vz, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(
                positionX, positionY, positionZ, vx, vy, vz, new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME)));

        // test constructor with position coordinates, speed coordinates and coordinate transformation
        // matrix
        frame = new ECIFrame(positionX, positionY, positionZ, speedX, speedY, speedZ, c1);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);

        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);

        assertEquals(x, frame.getPositionX().getValue().doubleValue(), 0.0);
        assertEquals(y, frame.getPositionY().getValue().doubleValue(), 0.0);
        assertEquals(z, frame.getPositionZ().getValue().doubleValue(), 0.0);

        assertEquals(vx, frame.getSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(vy, frame.getSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(vz, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        c2 = frame.getCoordinateTransformation();
        assertEquals(c1, c2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> new ECIFrame(
                positionX, positionY, positionZ, speedX, speedY, speedZ, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));

        // test constructor with another ECEF frame
        frame = new ECIFrame(x, y, z, vx, vy, vz, c1);
        final var frame2 = new ECIFrame(frame);

        // check
        assertEquals(frame.getX(), frame2.getX(), 0.0);
        assertEquals(frame.getY(), frame2.getY(), 0.0);
        assertEquals(frame.getZ(), frame2.getZ(), 0.0);

        assertEquals(frame.getVx(), frame2.getVx(), 0.0);
        assertEquals(frame.getVy(), frame2.getVy(), 0.0);
        assertEquals(frame.getVz(), frame2.getVz(), 0.0);

        assertEquals(frame.getCoordinateTransformation(), frame2.getCoordinateTransformation());
    }

    @Test
    void testGetSetX() {

        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getX(), 0.0);

        // set new value
        frame.setX(x);

        // check
        assertEquals(x, frame.getX(), 0.0);
    }

    @Test
    void testGetSetY() {

        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getY(), 0.0);

        // set new value
        frame.setY(y);

        // check
        assertEquals(y, frame.getY(), 0.0);
    }

    @Test
    void testGetSetZ() {

        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getZ(), 0.0);

        // set new value
        frame.setZ(z);

        // check
        assertEquals(z, frame.getZ(), 0.0);
    }

    @Test
    void testSetCoordinates() {

        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var frame = new ECIFrame();

        // check initial values
        assertEquals(0.0, frame.getX(), 0.0);
        assertEquals(0.0, frame.getY(), 0.0);
        assertEquals(0.0, frame.getZ(), 0.0);

        // set new values
        frame.setCoordinates(x, y, z);

        // check
        assertEquals(x, frame.getX(), 0.0);
        assertEquals(y, frame.getY(), 0.0);
        assertEquals(z, frame.getZ(), 0.0);
    }

    @Test
    void testGetSetPosition() {

        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(frame.getPosition(), Point3D.create());

        // set new value
        final var position = new InhomogeneousPoint3D(x, y, z);
        frame.setPosition(position);

        // check
        assertEquals(frame.getPosition(), position);

        final var position2 = new InhomogeneousPoint3D();
        frame.getPosition(position2);
        assertEquals(position, position2);
    }

    @Test
    void testGetSetPositionX() {

        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getPositionX().getValue().doubleValue(), 0.0);

        // set new value
        final var positionX1 = new Distance(x, DistanceUnit.METER);
        frame.setPositionX(positionX1);

        // check
        final var positionX2 = new Distance(0.0, DistanceUnit.CENTIMETER);
        frame.getPositionX(positionX2);
        final var positionX3 = frame.getPositionX();

        assertEquals(positionX1, positionX2);
        assertEquals(positionX1, positionX3);
    }

    @Test
    void testGetSetPositionY() {

        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getPositionY().getValue().doubleValue(), 0.0);

        // set new value
        final var positionY1 = new Distance(y, DistanceUnit.METER);
        frame.setPositionY(positionY1);

        // check
        final var positionY2 = new Distance(0.0, DistanceUnit.CENTIMETER);
        frame.getPositionY(positionY2);
        final var positionY3 = frame.getPositionY();

        assertEquals(positionY1, positionY2);
        assertEquals(positionY1, positionY3);
    }

    @Test
    void testGetSetPositionZ() {

        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getPositionZ().getValue().doubleValue(), 0.0);

        // set new value
        final var positionZ1 = new Distance(z, DistanceUnit.METER);
        frame.setPositionZ(positionZ1);

        // check
        final var positionZ2 = new Distance(0.0, DistanceUnit.CENTIMETER);
        frame.getPositionZ(positionZ2);
        final var positionZ3 = frame.getPositionZ();

        assertEquals(positionZ1, positionZ2);
        assertEquals(positionZ1, positionZ3);
    }

    @Test
    void testSetPositionCoordinates() {

        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var positionX = new Distance(x, DistanceUnit.METER);
        final var positionY = new Distance(y, DistanceUnit.METER);
        final var positionZ = new Distance(z, DistanceUnit.METER);

        final var frame = new ECIFrame();

        frame.setPositionCoordinates(positionX, positionY, positionZ);

        // check
        assertEquals(positionX, frame.getPositionX());
        assertEquals(positionY, frame.getPositionY());
        assertEquals(positionZ, frame.getPositionZ());
    }

    @Test
    void testGetPositionNorm() {

        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var norm = Math.sqrt(Math.pow(x, 2.0) + Math.pow(y, 2.0) + Math.pow(z, 2.0));

        final var frame = new ECIFrame(x, y, z);

        assertEquals(frame.getPositionNorm(), norm, ABSOLUTE_ERROR);

        final var normDistance1 = new Distance(0.0, DistanceUnit.KILOMETER);
        frame.getPositionNormAsDistance(normDistance1);
        final var normDistance2 = frame.getPositionNormAsDistance();

        assertEquals(norm, normDistance1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(DistanceUnit.METER, normDistance1.getUnit());
        assertEquals(normDistance1, normDistance2);
    }

    @Test
    void testGetSetVx() {

        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getVx(), 0.0);

        // set new value
        frame.setVx(vx);

        // check
        assertEquals(vx, frame.getVx(), 0.0);
    }

    @Test
    void testGetSetVy() {

        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getVy(), 0.0);

        // set new value
        frame.setVy(vy);

        // check
        assertEquals(vy, frame.getVy(), 0.0);
    }

    @Test
    void testGetSetVz() {

        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getVz(), 0.0);

        // set new value
        frame.setVz(vz);

        // check
        assertEquals(vz, frame.getVz(), 0.0);
    }

    @Test
    void testSetVelocityCoordinates() {

        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getVx(), 0.0);
        assertEquals(0.0, frame.getVy(), 0.0);
        assertEquals(0.0, frame.getVz(), 0.0);

        // set new values
        frame.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(vx, frame.getVx(), 0.0);
        assertEquals(vy, frame.getVy(), 0.0);
        assertEquals(vz, frame.getVz(), 0.0);
    }

    @Test
    void testGetVelocityNorm() {

        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var norm = Math.sqrt(Math.pow(vx, 2.0) + Math.pow(vy, 2.0) + Math.pow(vz, 2.0));

        final var frame = new ECIFrame();
        frame.setVelocityCoordinates(vx, vy, vz);

        assertEquals(norm, frame.getVelocityNorm(), ABSOLUTE_ERROR);

        final var normSpeed1 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getVelocityNormAsSpeed(normSpeed1);
        final var normSpeed2 = frame.getVelocityNormAsSpeed();

        assertEquals(norm, normSpeed1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(SpeedUnit.METERS_PER_SECOND, normSpeed1.getUnit());
        assertEquals(normSpeed1, normSpeed2);
    }

    @Test
    void testGetSetSpeedX() {

        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getSpeedX().getValue().doubleValue(), 0.0);

        // set new value
        final var speedX1 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedX(speedX1);

        // check
        final var speedX2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedX(speedX2);
        final var speedX3 = frame.getSpeedX();

        assertEquals(speedX1, speedX2);
        assertEquals(speedX1, speedX3);
    }

    @Test
    void testGetSetSpeedY() {

        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getSpeedY().getValue().doubleValue(), 0.0);

        // set new value
        final var speedY1 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedY(speedY1);

        // check
        final var speedY2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedY(speedY2);
        final var speedY3 = frame.getSpeedY();

        assertEquals(speedY1, speedY2);
        assertEquals(speedY1, speedY3);
    }

    @Test
    void testGetSetSpeedZ() {

        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var frame = new ECIFrame();

        // check initial value
        assertEquals(0.0, frame.getSpeedZ().getValue().doubleValue(), 0.0);

        // set new value
        final var speedZ1 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        frame.setSpeedZ(speedZ1);

        // check
        final var speedZ2 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        frame.getSpeedZ(speedZ2);
        final var speedZ3 = frame.getSpeedZ();

        assertEquals(speedZ1, speedZ2);
        assertEquals(speedZ1, speedZ3);
    }

    @Test
    void testSetSpeedCoordinates() {

        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var frame = new ECIFrame();

        // set new values
        frame.setSpeedCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(speedX, frame.getSpeedX());
        assertEquals(speedY, frame.getSpeedY());
        assertEquals(speedZ, frame.getSpeedZ());
    }

    @Test
    void testGetSetCoordinateTransformation() throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final var frame = new ECIFrame();

        // check initial value
        final var c1 = frame.getCoordinateTransformation();
        assertEquals(FrameType.BODY_FRAME, c1.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_INERTIAL_FRAME, c1.getDestinationType());
        assertEquals(Matrix.identity(3, 3), c1.getMatrix());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c2 = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

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
        final var c = new CoordinateTransformation(m1, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame = new ECIFrame(c);

        // check
        assertEquals(frame.getCoordinateTransformationMatrix(), m1);
        final var m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        frame.getCoordinateTransformationMatrix(m2);
        assertEquals(m1, m2);
    }

    @Test
    void testGetSetCoordinateTransformationMatrix2() throws InvalidSourceAndDestinationFrameTypeException,
            WrongSizeException, InvalidRotationMatrixException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m1 = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame = new ECIFrame(c);

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
    void testGetSetCoordinateTransformationMatrix3() throws InvalidSourceAndDestinationFrameTypeException,
            WrongSizeException, InvalidRotationMatrixException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m1 = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame = new ECIFrame(c);

        // check default value
        assertEquals(frame.getCoordinateTransformationMatrix(),
                Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS));
        final var m2 = new Matrix(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
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
    void testGetSetCoordinateTransformationRotation() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame = new ECIFrame(c);

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

        final var c1 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        final var c2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final var c3 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        assertTrue(ECIFrame.isValidCoordinateTransformation(c1));
        assertFalse(ECIFrame.isValidCoordinateTransformation(c2));
        assertFalse(ECIFrame.isValidCoordinateTransformation(c3));
    }

    @Test
    void testCopyTo() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame1 = new ECIFrame(x, y, z, vx, vy, vz, c);
        final var frame2 = new ECIFrame();
        frame1.copyTo(frame2);

        // check
        assertEquals(frame1.getX(), frame2.getX(), 0.0);
        assertEquals(frame1.getY(), frame2.getY(), 0.0);
        assertEquals(frame1.getZ(), frame2.getZ(), 0.0);
        assertEquals(frame1.getVx(), frame2.getVx(), 0.0);
        assertEquals(frame1.getVy(), frame2.getVy(), 0.0);
        assertEquals(frame1.getVz(), frame2.getVz(), 0.0);
        assertEquals(frame1.getCoordinateTransformation(), frame2.getCoordinateTransformation());
    }

    @Test
    void testCopyFrom() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame1 = new ECIFrame(x, y, z, vx, vy, vz, c);
        final var frame2 = new ECIFrame();
        frame2.copyFrom(frame1);

        // check
        assertEquals(frame1.getX(), frame2.getX(), 0.0);
        assertEquals(frame1.getY(), frame2.getY(), 0.0);
        assertEquals(frame1.getZ(), frame2.getZ(), 0.0);
        assertEquals(frame1.getVx(), frame2.getVx(), 0.0);
        assertEquals(frame1.getVy(), frame2.getVy(), 0.0);
        assertEquals(frame1.getVz(), frame2.getVz(), 0.0);
        assertEquals(frame1.getCoordinateTransformation(), frame2.getCoordinateTransformation());
    }

    @Test
    void testHashCode() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame1 = new ECIFrame(x, y, z, vx, vy, vz, c);
        final var frame2 = new ECIFrame(x, y, z, vx, vy, vz, c);
        final var frame3 = new ECIFrame();

        assertEquals(frame1.hashCode(), frame2.hashCode());
        assertNotEquals(frame1.hashCode(), frame3.hashCode());
    }

    @Test
    void testEquals() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame1 = new ECIFrame(x, y, z, vx, vy, vz, c);
        final var frame2 = new ECIFrame(x, y, z, vx, vy, vz, c);
        final var frame3 = new ECIFrame();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(frame1.equals((Object) frame1));
        //noinspection EqualsWithItself
        assertTrue(frame1.equals(frame1));
        assertEquals(frame1, frame2);
        assertNotEquals(frame1, frame3);
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

        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame1 = new ECIFrame(x, y, z, vx, vy, vz, c);
        final var frame2 = new ECIFrame(x, y, z, vx, vy, vz, c);
        final var frame3 = new ECIFrame();

        assertTrue(frame1.equals(frame1, THRESHOLD));
        assertTrue(frame1.equals(frame2, THRESHOLD));
        assertFalse(frame1.equals(frame3, THRESHOLD));
        assertFalse(frame1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            CloneNotSupportedException {

        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

        final var frame2 = frame1.clone();

        assertEquals(frame1, frame2);
    }

    @Test
    void testSerializeDeserialize() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var vx = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vy = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vz = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

        final var frame1 = new ECIFrame(x, y, z, vx, vy, vz, c);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(frame1);
        final var frame2 = SerializationHelper.<ECIFrame>deserialize(bytes);

        // check
        assertEquals(frame1, frame2);
        assertNotSame(frame1, frame2);
    }
}
