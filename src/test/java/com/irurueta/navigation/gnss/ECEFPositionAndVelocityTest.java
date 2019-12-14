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
package com.irurueta.navigation.gnss;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ECEFPositionAndVelocityTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SPEED_VALUE = -0.1;
    private static final double MAX_SPEED_VALUE = 0.1;

    @Test
    public void testConstructor() {
        // test empty constructor
        ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default values
        assertEquals(positionAndVelocity.getX(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getY(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getZ(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with position coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);

        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with position distance
        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        positionAndVelocity = new ECEFPositionAndVelocity(
                distanceX, distanceY, distanceZ);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with ECEF position
        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        positionAndVelocity = new ECEFPositionAndVelocity(
                ecefPosition);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with position
        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        positionAndVelocity = new ECEFPositionAndVelocity(
                position);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(positionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with speed coordinates
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX = new Speed(vx,
                SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy,
                SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz,
                SpeedUnit.METERS_PER_SECOND);

        positionAndVelocity = new ECEFPositionAndVelocity(
                speedX, speedY, speedZ);

        // check default values
        assertEquals(positionAndVelocity.getX(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getY(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getZ(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with ECEF velocity
        final ECEFVelocity ecefVelocity = new ECEFVelocity(
                vx, vy, vz);

        positionAndVelocity = new ECEFPositionAndVelocity(
                ecefVelocity);

        // check default values
        assertEquals(positionAndVelocity.getX(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getY(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getZ(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z,
                vx, vy, vz);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z,
                speedX, speedY, speedZ);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position coordinates and ECEF velocity
        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z,
                ecefVelocity);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position distance and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(
                distanceX, distanceY, distanceZ, vx, vy, vz);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position distance and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(
                distanceX, distanceY, distanceZ, speedX, speedY, speedZ);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position distance and ECEF velocity
        positionAndVelocity = new ECEFPositionAndVelocity(
                distanceX, distanceY, distanceZ, ecefVelocity);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with ECEF position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition,
                vx, vy, vz);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with ECEF position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition,
                speedX, speedY, speedZ);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with ECEF position and velocity
        positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition,
                ecefVelocity);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(position,
                vx, vy, vz);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(position,
                speedX, speedY, speedZ);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and ECEF velocity
        positionAndVelocity = new ECEFPositionAndVelocity(position,
                ecefVelocity);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);


        // test copy constructor
        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z,
                vx, vy, vz);

        final ECEFPositionAndVelocity positionAndVelocity2 =
                new ECEFPositionAndVelocity(positionAndVelocity);

        // check default values
        assertEquals(positionAndVelocity2.getX(), x, 0.0);
        assertEquals(positionAndVelocity2.getY(), y, 0.0);
        assertEquals(positionAndVelocity2.getZ(), z, 0.0);
        assertEquals(positionAndVelocity2.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity2.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity2.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetX() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        assertEquals(positionAndVelocity.getX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        positionAndVelocity.setX(x);

        // check
        assertEquals(positionAndVelocity.getX(), x, 0.0);
    }

    @Test
    public void testGetSetY() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        assertEquals(positionAndVelocity.getY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        positionAndVelocity.setY(y);

        // check
        assertEquals(positionAndVelocity.getY(), y, 0.0);
    }

    @Test
    public void testGetSetZ() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        assertEquals(positionAndVelocity.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        positionAndVelocity.setZ(z);

        // check
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
    }

    @Test
    public void testSetPositionCoordinates() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default values
        assertEquals(positionAndVelocity.getX(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getY(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        positionAndVelocity.setPositionCoordinates(x, y, z);

        // check
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetXDistance() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final Distance distanceX1 = positionAndVelocity.getXDistance();

        assertEquals(distanceX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceX1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);

        positionAndVelocity.setXDistance(distanceX2);

        // check
        final Distance distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        positionAndVelocity.getXDistance(distanceX3);
        final Distance distanceX4 = positionAndVelocity.getXDistance();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    public void testGetSetYDistance() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final Distance distanceY1 = positionAndVelocity.getYDistance();

        assertEquals(distanceY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceY1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);

        positionAndVelocity.setYDistance(distanceY2);

        // check
        final Distance distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        positionAndVelocity.getYDistance(distanceY3);
        final Distance distanceY4 = positionAndVelocity.getYDistance();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    public void testGetSetZDistance() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final Distance distanceZ1 = positionAndVelocity.getZDistance();

        assertEquals(distanceZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceZ1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);

        positionAndVelocity.setZDistance(distanceZ2);

        // check
        final Distance distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        positionAndVelocity.getZDistance(distanceZ3);
        final Distance distanceZ4 = positionAndVelocity.getZDistance();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    public void testSetPositionDistanceCoordinates() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        assertEquals(positionAndVelocity.getX(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getY(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        positionAndVelocity.setPositionDistanceCoordinates(
                distanceX, distanceY, distanceZ);

        // check
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetEcefPosition() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final ECEFPosition ecefPosition1 = positionAndVelocity
                .getEcefPosition();

        assertEquals(ecefPosition1.getX(), 0.0, 0.0);
        assertEquals(ecefPosition1.getY(), 0.0, 0.0);
        assertEquals(ecefPosition1.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final ECEFPosition ecefPosition2 = new ECEFPosition(x, y, z);
        positionAndVelocity.setEcefPosition(ecefPosition2);

        // check
        final ECEFPosition ecefPosition3 = new ECEFPosition();
        positionAndVelocity.getEcefPosition(ecefPosition3);
        final ECEFPosition ecefPosition4 = positionAndVelocity
                .getEcefPosition();

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    public void testGetSetPosition() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final Point3D position1 = positionAndVelocity.getPosition();

        assertEquals(position1.getInhomX(), 0.0, 0.0);
        assertEquals(position1.getInhomY(), 0.0, 0.0);
        assertEquals(position1.getInhomZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final Point3D position2 = new InhomogeneousPoint3D(x, y, z);
        positionAndVelocity.setPosition(position2);

        // check
        final Point3D position3 = new InhomogeneousPoint3D();
        positionAndVelocity.getPosition(position3);
        final Point3D position4 = positionAndVelocity.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetVx() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        assertEquals(positionAndVelocity.getVx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        positionAndVelocity.setVx(vx);

        // check
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
    }

    @Test
    public void testGetSetVy() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        assertEquals(positionAndVelocity.getVy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        positionAndVelocity.setVy(vy);

        // check
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
    }

    @Test
    public void testGetSetVz() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        assertEquals(positionAndVelocity.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        positionAndVelocity.setVz(vz);

        // check
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);
    }

    @Test
    public void testSetVelocityCoordinates() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default values
        assertEquals(positionAndVelocity.getVx(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getVy(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getVz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        positionAndVelocity.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetSpeedX() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final Speed speedX1 = positionAndVelocity.getSpeedX();

        assertEquals(speedX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedX1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        positionAndVelocity.setSpeedX(speedX2);

        // check
        final Speed speedX3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        positionAndVelocity.getSpeedX(speedX3);
        final Speed speedX4 = positionAndVelocity.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    public void testGetSetSpeedY() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final Speed speedY1 = positionAndVelocity.getSpeedY();

        assertEquals(speedY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedY1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        positionAndVelocity.setSpeedY(speedY2);

        // check
        final Speed speedY3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_SECOND);
        positionAndVelocity.getSpeedY(speedY3);
        final Speed speedY4 = positionAndVelocity.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    public void testGetSetSpeedZ() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final Speed speedZ1 = positionAndVelocity.getSpeedZ();

        assertEquals(speedZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedZ1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        positionAndVelocity.setSpeedZ(speedZ2);

        // check
        final Speed speedZ3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        positionAndVelocity.getSpeedZ(speedZ3);
        final Speed speedZ4 = positionAndVelocity.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    public void testSetSpeedCoordinates() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        assertEquals(positionAndVelocity.getVx(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getVy(), 0.0, 0.0);
        assertEquals(positionAndVelocity.getVz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        positionAndVelocity.setSpeedCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(positionAndVelocity.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetEcefVelocity() {
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity();

        // check default value
        final ECEFVelocity ecefVelocity1 = positionAndVelocity
                .getEcefVelocity();

        assertEquals(ecefVelocity1.getVx(), 0.0, 0.0);
        assertEquals(ecefVelocity1.getVy(), 0.0, 0.0);
        assertEquals(ecefVelocity1.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity(vx, vy, vz);

        positionAndVelocity.setEcefVelocity(ecefVelocity2);

        // check
        final ECEFVelocity ecefVelocity3 = new ECEFVelocity();
        positionAndVelocity.getEcefVelocity(ecefVelocity3);
        final ECEFVelocity ecefVelocity4 = positionAndVelocity
                .getEcefVelocity();

        assertEquals(ecefVelocity2, ecefVelocity3);
        assertEquals(ecefVelocity2, ecefVelocity4);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFPositionAndVelocity positionAndVelocity1 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final ECEFPositionAndVelocity positionAndVelocity2 =
                new ECEFPositionAndVelocity();

        positionAndVelocity1.copyTo(positionAndVelocity2);

        // check
        assertEquals(positionAndVelocity2.getX(), x, 0.0);
        assertEquals(positionAndVelocity2.getY(), y, 0.0);
        assertEquals(positionAndVelocity2.getZ(), z, 0.0);
        assertEquals(positionAndVelocity2.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity2.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity2.getVz(), vz, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFPositionAndVelocity positionAndVelocity1 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final ECEFPositionAndVelocity positionAndVelocity2 =
                new ECEFPositionAndVelocity();

        positionAndVelocity2.copyFrom(positionAndVelocity1);

        // check
        assertEquals(positionAndVelocity2.getX(), x, 0.0);
        assertEquals(positionAndVelocity2.getY(), y, 0.0);
        assertEquals(positionAndVelocity2.getZ(), z, 0.0);
        assertEquals(positionAndVelocity2.getVx(), vx, 0.0);
        assertEquals(positionAndVelocity2.getVy(), vy, 0.0);
        assertEquals(positionAndVelocity2.getVz(), vz, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFPositionAndVelocity positionAndVelocity1 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final ECEFPositionAndVelocity positionAndVelocity2 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final ECEFPositionAndVelocity positionAndVelocity3 =
                new ECEFPositionAndVelocity();

        // check
        assertEquals(positionAndVelocity1.hashCode(),
                positionAndVelocity2.hashCode());
        assertNotEquals(positionAndVelocity1.hashCode(),
                positionAndVelocity3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFPositionAndVelocity positionAndVelocity1 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final ECEFPositionAndVelocity positionAndVelocity2 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final ECEFPositionAndVelocity positionAndVelocity3 =
                new ECEFPositionAndVelocity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(positionAndVelocity1.equals((Object)positionAndVelocity1));
        assertTrue(positionAndVelocity1.equals(positionAndVelocity1));
        assertTrue(positionAndVelocity1.equals(positionAndVelocity2));
        assertFalse(positionAndVelocity1.equals(positionAndVelocity3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(positionAndVelocity1.equals((Object)null));
        assertFalse(positionAndVelocity1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(positionAndVelocity1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFPositionAndVelocity positionAndVelocity1 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final ECEFPositionAndVelocity positionAndVelocity2 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final ECEFPositionAndVelocity positionAndVelocity3 =
                new ECEFPositionAndVelocity();

        assertTrue(positionAndVelocity1.equals(positionAndVelocity1,
                THRESHOLD));
        assertTrue(positionAndVelocity1.equals(positionAndVelocity2,
                THRESHOLD));
        assertFalse(positionAndVelocity1.equals(positionAndVelocity3,
                THRESHOLD));
        assertFalse(positionAndVelocity1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFPositionAndVelocity positionAndVelocity1 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);

        final Object positionAndVelocity2 = positionAndVelocity1
                .clone();

        assertEquals(positionAndVelocity1, positionAndVelocity2);
    }
}
