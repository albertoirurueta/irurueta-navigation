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

public class SatellitePositionAndVelocityTest {

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
        SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with position coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE,
                MAX_POSITION_VALUE);

        satellitePositionAndVelocity = new SatellitePositionAndVelocity(x, y, z);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with position distance
        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        satellitePositionAndVelocity = new SatellitePositionAndVelocity(
                distanceX, distanceY, distanceZ);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with ECEF position
        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(
                ecefPosition);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(),
                0.0, 0.0);


        // test constructor with position
        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(
                position);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(),
                0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(),
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

        satellitePositionAndVelocity = new SatellitePositionAndVelocity(
                speedX, speedY, speedZ);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with ECEF velocity
        final ECEFVelocity ecefVelocity = new ECEFVelocity(
                vx, vy, vz);

        satellitePositionAndVelocity = new SatellitePositionAndVelocity(
                ecefVelocity);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and speed coordinates
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(x, y, z,
                vx, vy, vz);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and speed coordinates
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(x, y, z,
                speedX, speedY, speedZ);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position coordinates and ECEF velocity
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(x, y, z,
                ecefVelocity);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position distance and speed coordinates
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(
                distanceX, distanceY, distanceZ, vx, vy, vz);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position distance and speed coordinates
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(
                distanceX, distanceY, distanceZ, speedX, speedY, speedZ);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position distance and ECEF velocity
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(
                distanceX, distanceY, distanceZ, ecefVelocity);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with ECEF position and speed coordinates
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(ecefPosition,
                vx, vy, vz);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with ECEF position and speed coordinates
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(ecefPosition,
                speedX, speedY, speedZ);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with ECEF position and velocity
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(ecefPosition,
                ecefVelocity);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and speed coordinates
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(position,
                vx, vy, vz);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and speed coordinates
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(position,
                speedX, speedY, speedZ);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);


        // test constructor with position and ECEF velocity
        satellitePositionAndVelocity = new SatellitePositionAndVelocity(position,
                ecefVelocity);

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetX() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        assertEquals(satellitePositionAndVelocity.getX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        satellitePositionAndVelocity.setX(x);

        // check
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
    }

    @Test
    public void testGetSetY() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        assertEquals(satellitePositionAndVelocity.getY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        satellitePositionAndVelocity.setY(y);

        // check
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
    }

    @Test
    public void testGetSetZ() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        assertEquals(satellitePositionAndVelocity.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        satellitePositionAndVelocity.setZ(z);

        // check
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
    }

    @Test
    public void testSetPositionCoordinates() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default values
        assertEquals(satellitePositionAndVelocity.getX(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        satellitePositionAndVelocity.setPositionCoordinates(x, y, z);
    }

    @Test
    public void testGetSetXDistance() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final Distance distanceX1 = satellitePositionAndVelocity.getXDistance();

        assertEquals(distanceX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceX1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);

        satellitePositionAndVelocity.setXDistance(distanceX2);

        // check
        final Distance distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        satellitePositionAndVelocity.getXDistance(distanceX3);
        final Distance distanceX4 = satellitePositionAndVelocity.getXDistance();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    public void testGetSetYDistance() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final Distance distanceY1 = satellitePositionAndVelocity.getYDistance();

        assertEquals(distanceY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceY1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);

        satellitePositionAndVelocity.setYDistance(distanceY2);

        // check
        final Distance distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        satellitePositionAndVelocity.getYDistance(distanceY3);
        final Distance distanceY4 = satellitePositionAndVelocity.getYDistance();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    public void testGetSetZDistance() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final Distance distanceZ1 = satellitePositionAndVelocity.getZDistance();

        assertEquals(distanceZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceZ1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);

        satellitePositionAndVelocity.setZDistance(distanceZ2);

        // check
        final Distance distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        satellitePositionAndVelocity.getZDistance(distanceZ3);
        final Distance distanceZ4 = satellitePositionAndVelocity.getZDistance();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    public void testSetPositionDistanceCoordinates() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        assertEquals(satellitePositionAndVelocity.getX(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        satellitePositionAndVelocity.setPositionDistanceCoordinates(
                distanceX, distanceY, distanceZ);

        // check
        assertEquals(satellitePositionAndVelocity.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetEcefPosition() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final ECEFPosition ecefPosition1 = satellitePositionAndVelocity
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
        satellitePositionAndVelocity.setEcefPosition(ecefPosition2);

        // check
        final ECEFPosition ecefPosition3 = new ECEFPosition();
        satellitePositionAndVelocity.getEcefPosition(ecefPosition3);
        final ECEFPosition ecefPosition4 = satellitePositionAndVelocity
                .getEcefPosition();

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    public void testGetSetPosition() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final Point3D position1 = satellitePositionAndVelocity.getPosition();

        assertEquals(position1.getInhomX(), 0.0, 0.0);
        assertEquals(position1.getInhomY(), 0.0, 0.0);
        assertEquals(position1.getInhomZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final Point3D position2 = new InhomogeneousPoint3D(x, y, z);
        satellitePositionAndVelocity.setPosition(position2);

        // check
        final Point3D position3 = new InhomogeneousPoint3D();
        satellitePositionAndVelocity.getPosition(position3);
        final Point3D position4 = satellitePositionAndVelocity.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetVx() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        assertEquals(satellitePositionAndVelocity.getVx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        satellitePositionAndVelocity.setVx(vx);

        // check
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
    }

    @Test
    public void testGetSetVy() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        assertEquals(satellitePositionAndVelocity.getVy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        satellitePositionAndVelocity.setVy(vy);

        // check
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
    }

    @Test
    public void testGetSetVz() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        assertEquals(satellitePositionAndVelocity.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        satellitePositionAndVelocity.setVz(vz);

        // check
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);
    }

    @Test
    public void testSetVelocityCoordinates() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default values
        assertEquals(satellitePositionAndVelocity.getVx(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        satellitePositionAndVelocity.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetSpeedX() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final Speed speedX1 = satellitePositionAndVelocity.getSpeedX();

        assertEquals(speedX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedX1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        satellitePositionAndVelocity.setSpeedX(speedX2);

        // check
        final Speed speedX3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        satellitePositionAndVelocity.getSpeedX(speedX3);
        final Speed speedX4 = satellitePositionAndVelocity.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    public void testGetSetSpeedY() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final Speed speedY1 = satellitePositionAndVelocity.getSpeedY();

        assertEquals(speedY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedY1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        satellitePositionAndVelocity.setSpeedY(speedY2);

        // check
        final Speed speedY3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_SECOND);
        satellitePositionAndVelocity.getSpeedY(speedY3);
        final Speed speedY4 = satellitePositionAndVelocity.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    public void testGetSetSpeedZ() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final Speed speedZ1 = satellitePositionAndVelocity.getSpeedZ();

        assertEquals(speedZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedZ1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        satellitePositionAndVelocity.setSpeedZ(speedZ2);

        // check
        final Speed speedZ3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        satellitePositionAndVelocity.getSpeedZ(speedZ3);
        final Speed speedZ4 = satellitePositionAndVelocity.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    public void testSetSpeedCoordinates() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        assertEquals(satellitePositionAndVelocity.getVx(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), 0.0, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        satellitePositionAndVelocity.setSpeedCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(satellitePositionAndVelocity.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetEcefVelocity() {
        final SatellitePositionAndVelocity satellitePositionAndVelocity =
                new SatellitePositionAndVelocity();

        // check default value
        final ECEFVelocity ecefVelocity1 = satellitePositionAndVelocity
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

        satellitePositionAndVelocity.setEcefVelocity(ecefVelocity2);

        // check
        final ECEFVelocity ecefVelocity3 = new ECEFVelocity();
        satellitePositionAndVelocity.getEcefVelocity(ecefVelocity3);
        final ECEFVelocity ecefVelocity4 = satellitePositionAndVelocity
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

        final SatellitePositionAndVelocity satellitePositionAndVelocity1 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);
        final SatellitePositionAndVelocity satellitePositionAndVelocity2 =
                new SatellitePositionAndVelocity();

        satellitePositionAndVelocity1.copyTo(satellitePositionAndVelocity2);

        // check
        assertEquals(satellitePositionAndVelocity2.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity2.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity2.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity2.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity2.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity2.getVz(), vz, 0.0);
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

        final SatellitePositionAndVelocity satellitePositionAndVelocity1 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);
        final SatellitePositionAndVelocity satellitePositionAndVelocity2 =
                new SatellitePositionAndVelocity();

        satellitePositionAndVelocity2.copyFrom(satellitePositionAndVelocity1);

        // check
        assertEquals(satellitePositionAndVelocity2.getX(), x, 0.0);
        assertEquals(satellitePositionAndVelocity2.getY(), y, 0.0);
        assertEquals(satellitePositionAndVelocity2.getZ(), z, 0.0);
        assertEquals(satellitePositionAndVelocity2.getVx(), vx, 0.0);
        assertEquals(satellitePositionAndVelocity2.getVy(), vy, 0.0);
        assertEquals(satellitePositionAndVelocity2.getVz(), vz, 0.0);
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

        final SatellitePositionAndVelocity satellitePositionAndVelocity1 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);
        final SatellitePositionAndVelocity satellitePositionAndVelocity2 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);
        final SatellitePositionAndVelocity satellitePositionAndVelocity3 =
                new SatellitePositionAndVelocity();

        // check
        assertEquals(satellitePositionAndVelocity1.hashCode(),
                satellitePositionAndVelocity2.hashCode());
        assertNotEquals(satellitePositionAndVelocity1.hashCode(),
                satellitePositionAndVelocity3.hashCode());
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

        final SatellitePositionAndVelocity satellitePositionAndVelocity1 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);
        final SatellitePositionAndVelocity satellitePositionAndVelocity2 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);
        final SatellitePositionAndVelocity satellitePositionAndVelocity3 =
                new SatellitePositionAndVelocity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(satellitePositionAndVelocity1.equals((Object)satellitePositionAndVelocity1));
        assertTrue(satellitePositionAndVelocity1.equals(satellitePositionAndVelocity1));
        assertTrue(satellitePositionAndVelocity1.equals(satellitePositionAndVelocity2));
        assertFalse(satellitePositionAndVelocity1.equals(satellitePositionAndVelocity3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(satellitePositionAndVelocity1.equals((Object)null));
        assertFalse(satellitePositionAndVelocity1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(satellitePositionAndVelocity1.equals(new Object()));
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

        final SatellitePositionAndVelocity satellitePositionAndVelocity1 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);
        final SatellitePositionAndVelocity satellitePositionAndVelocity2 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);
        final SatellitePositionAndVelocity satellitePositionAndVelocity3 =
                new SatellitePositionAndVelocity();

        assertTrue(satellitePositionAndVelocity1.equals(satellitePositionAndVelocity1,
                THRESHOLD));
        assertTrue(satellitePositionAndVelocity1.equals(satellitePositionAndVelocity2,
                THRESHOLD));
        assertFalse(satellitePositionAndVelocity1.equals(satellitePositionAndVelocity3,
                THRESHOLD));
        assertFalse(satellitePositionAndVelocity1.equals(null, THRESHOLD));
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

        final SatellitePositionAndVelocity satellitePositionAndVelocity1 =
                new SatellitePositionAndVelocity(x, y, z, vx, vy, vz);

        final Object satellitePositionAndVelocity2 = satellitePositionAndVelocity1
                .clone();

        assertEquals(satellitePositionAndVelocity1, satellitePositionAndVelocity2);
    }
}
