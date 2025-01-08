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
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class ECEFPositionAndVelocityTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SPEED_VALUE = -0.1;
    private static final double MAX_SPEED_VALUE = 0.1;

    @Test
    void testConstructor() {
        // test empty constructor
        var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default values
        assertEquals(0.0, positionAndVelocity.getX(), 0.0);
        assertEquals(0.0, positionAndVelocity.getY(), 0.0);
        assertEquals(0.0, positionAndVelocity.getZ(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVz(), 0.0);

        // test constructor with position coordinates
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z);

        // check default values
        assertEquals(positionAndVelocity.getX(), x, 0.0);
        assertEquals(positionAndVelocity.getY(), y, 0.0);
        assertEquals(positionAndVelocity.getZ(), z, 0.0);
        assertEquals(0.0, positionAndVelocity.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVz(), 0.0);

        // test constructor with position distance
        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);

        positionAndVelocity = new ECEFPositionAndVelocity(distanceX, distanceY, distanceZ);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVz(), 0.0);

        // test constructor with ECEF position
        final var ecefPosition = new ECEFPosition(x, y, z);
        positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVz(), 0.0);

        // test constructor with position
        final var position = new InhomogeneousPoint3D(x, y, z);
        positionAndVelocity = new ECEFPositionAndVelocity(position);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y,positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVz(), 0.0);

        // test constructor with speed coordinates
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        positionAndVelocity = new ECEFPositionAndVelocity(speedX, speedY, speedZ);

        // check default values
        assertEquals(0.0, positionAndVelocity.getX(), 0.0);
        assertEquals(0.0, positionAndVelocity.getY(), 0.0);
        assertEquals(0.0, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with ECEF velocity
        final var ecefVelocity = new ECEFVelocity(vx, vy, vz);

        positionAndVelocity = new ECEFPositionAndVelocity(ecefVelocity);

        // check default values
        assertEquals(0.0, positionAndVelocity.getX(), 0.0);
        assertEquals(0.0, positionAndVelocity.getY(), 0.0);
        assertEquals(0.0, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z, speedX, speedY, speedZ);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position coordinates and ECEF velocity
        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z, ecefVelocity);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position distance and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(distanceX, distanceY, distanceZ, vx, vy, vz);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position distance and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(distanceX, distanceY, distanceZ, speedX, speedY, speedZ);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position distance and ECEF velocity
        positionAndVelocity = new ECEFPositionAndVelocity(distanceX, distanceY, distanceZ, ecefVelocity);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with ECEF position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition, vx, vy, vz);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with ECEF position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition, speedX, speedY, speedZ);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with ECEF position and velocity
        positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition, ecefVelocity);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(position, vx, vy, vz);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position and speed coordinates
        positionAndVelocity = new ECEFPositionAndVelocity(position, speedX, speedY, speedZ);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test constructor with position and ECEF velocity
        positionAndVelocity = new ECEFPositionAndVelocity(position, ecefVelocity);

        // check default values
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);

        // test copy constructor
        positionAndVelocity = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);

        final var positionAndVelocity2 = new ECEFPositionAndVelocity(positionAndVelocity);

        // check default values
        assertEquals(x, positionAndVelocity2.getX(), 0.0);
        assertEquals(y, positionAndVelocity2.getY(), 0.0);
        assertEquals(z, positionAndVelocity2.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity2.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity2.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity2.getVz(), 0.0);
    }

    @Test
    void testGetSetX() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        assertEquals(0.0, positionAndVelocity.getX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        positionAndVelocity.setX(x);

        // check
        assertEquals(x, positionAndVelocity.getX(), 0.0);
    }

    @Test
    void testGetSetY() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        assertEquals(0.0, positionAndVelocity.getY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        positionAndVelocity.setY(y);

        // check
        assertEquals(y, positionAndVelocity.getY(), 0.0);
    }

    @Test
    void testGetSetZ() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        assertEquals(0.0, positionAndVelocity.getZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        positionAndVelocity.setZ(z);

        // check
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
    }

    @Test
    void testSetPositionCoordinates() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default values
        assertEquals(0.0, positionAndVelocity.getX(), 0.0);
        assertEquals(0.0, positionAndVelocity.getY(), 0.0);
        assertEquals(0.0, positionAndVelocity.getZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        positionAndVelocity.setPositionCoordinates(x, y, z);

        // check
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
    }

    @Test
    void testGetSetXDistance() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var distanceX1 = positionAndVelocity.getXDistance();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var distanceX2 = new Distance(x, DistanceUnit.METER);

        positionAndVelocity.setXDistance(distanceX2);

        // check
        final var distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        positionAndVelocity.getXDistance(distanceX3);
        final var distanceX4 = positionAndVelocity.getXDistance();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    void testGetSetYDistance() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var distanceY1 = positionAndVelocity.getYDistance();

        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var distanceY2 = new Distance(y, DistanceUnit.METER);

        positionAndVelocity.setYDistance(distanceY2);

        // check
        final var distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        positionAndVelocity.getYDistance(distanceY3);
        final var distanceY4 = positionAndVelocity.getYDistance();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    void testGetSetZDistance() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var distanceZ1 = positionAndVelocity.getZDistance();

        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var distanceZ2 = new Distance(z, DistanceUnit.METER);

        positionAndVelocity.setZDistance(distanceZ2);

        // check
        final var distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        positionAndVelocity.getZDistance(distanceZ3);
        final var distanceZ4 = positionAndVelocity.getZDistance();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    void testSetPositionDistanceCoordinates() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        assertEquals(0.0, positionAndVelocity.getX(), 0.0);
        assertEquals(0.0, positionAndVelocity.getY(), 0.0);
        assertEquals(0.0, positionAndVelocity.getZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);

        positionAndVelocity.setPositionDistanceCoordinates(distanceX, distanceY, distanceZ);

        // check
        assertEquals(x, positionAndVelocity.getX(), 0.0);
        assertEquals(y, positionAndVelocity.getY(), 0.0);
        assertEquals(z, positionAndVelocity.getZ(), 0.0);
    }

    @Test
    void testGetSetEcefPosition() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var ecefPosition1 = positionAndVelocity.getEcefPosition();

        assertEquals(0.0, ecefPosition1.getX(), 0.0);
        assertEquals(0.0, ecefPosition1.getY(), 0.0);
        assertEquals(0.0, ecefPosition1.getZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var ecefPosition2 = new ECEFPosition(x, y, z);
        positionAndVelocity.setEcefPosition(ecefPosition2);

        // check
        final var ecefPosition3 = new ECEFPosition();
        positionAndVelocity.getEcefPosition(ecefPosition3);
        final var ecefPosition4 = positionAndVelocity.getEcefPosition();

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    void testGetSetPosition() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var position1 = positionAndVelocity.getPosition();

        assertEquals(0.0, position1.getInhomX(), 0.0);
        assertEquals(0.0, position1.getInhomY(), 0.0);
        assertEquals(0.0, position1.getInhomZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var position2 = new InhomogeneousPoint3D(x, y, z);
        positionAndVelocity.setPosition(position2);

        // check
        final var position3 = new InhomogeneousPoint3D();
        positionAndVelocity.getPosition(position3);
        final var position4 = positionAndVelocity.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetVx() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        assertEquals(0.0, positionAndVelocity.getVx(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        positionAndVelocity.setVx(vx);

        // check
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
    }

    @Test
    void testGetSetVy() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        assertEquals(0.0, positionAndVelocity.getVy(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        positionAndVelocity.setVy(vy);

        // check
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
    }

    @Test
    void testGetSetVz() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        assertEquals(0.0, positionAndVelocity.getVz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        positionAndVelocity.setVz(vz);

        // check
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);
    }

    @Test
    void testSetVelocityCoordinates() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default values
        assertEquals(0.0, positionAndVelocity.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        positionAndVelocity.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);
    }

    @Test
    void testGetSetSpeedX() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var speedX1 = positionAndVelocity.getSpeedX();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        positionAndVelocity.setSpeedX(speedX2);

        // check
        final var speedX3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        positionAndVelocity.getSpeedX(speedX3);
        final var speedX4 = positionAndVelocity.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    void testGetSetSpeedY() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var speedY1 = positionAndVelocity.getSpeedY();

        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        positionAndVelocity.setSpeedY(speedY2);

        // check
        final var speedY3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_SECOND);
        positionAndVelocity.getSpeedY(speedY3);
        final var speedY4 = positionAndVelocity.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    void testGetSetSpeedZ() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var speedZ1 = positionAndVelocity.getSpeedZ();

        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        positionAndVelocity.setSpeedZ(speedZ2);

        // check
        final var speedZ3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        positionAndVelocity.getSpeedZ(speedZ3);
        final var speedZ4 = positionAndVelocity.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    void testSetSpeedCoordinates() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        assertEquals(0.0, positionAndVelocity.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity.getVz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        positionAndVelocity.setSpeedCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(vx, positionAndVelocity.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity.getVz(), 0.0);
    }

    @Test
    void testGetSetEcefVelocity() {
        final var positionAndVelocity = new ECEFPositionAndVelocity();

        // check default value
        final var ecefVelocity1 = positionAndVelocity.getEcefVelocity();

        assertEquals(0.0, ecefVelocity1.getVx(), 0.0);
        assertEquals(0.0, ecefVelocity1.getVy(), 0.0);
        assertEquals(0.0, ecefVelocity1.getVz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var ecefVelocity2 = new ECEFVelocity(vx, vy, vz);

        positionAndVelocity.setEcefVelocity(ecefVelocity2);

        // check
        final var ecefVelocity3 = new ECEFVelocity();
        positionAndVelocity.getEcefVelocity(ecefVelocity3);
        final var ecefVelocity4 = positionAndVelocity.getEcefVelocity();

        assertEquals(ecefVelocity2, ecefVelocity3);
        assertEquals(ecefVelocity2, ecefVelocity4);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var positionAndVelocity1 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final var positionAndVelocity2 = new ECEFPositionAndVelocity();

        positionAndVelocity1.copyTo(positionAndVelocity2);

        // check
        assertEquals(x, positionAndVelocity2.getX(), 0.0);
        assertEquals(y, positionAndVelocity2.getY(), 0.0);
        assertEquals(z, positionAndVelocity2.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity2.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity2.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity2.getVz(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var positionAndVelocity1 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final var positionAndVelocity2 = new ECEFPositionAndVelocity();

        positionAndVelocity2.copyFrom(positionAndVelocity1);

        // check
        assertEquals(x, positionAndVelocity2.getX(), 0.0);
        assertEquals(y, positionAndVelocity2.getY(), 0.0);
        assertEquals(z, positionAndVelocity2.getZ(), 0.0);
        assertEquals(vx, positionAndVelocity2.getVx(), 0.0);
        assertEquals(vy, positionAndVelocity2.getVy(), 0.0);
        assertEquals(vz, positionAndVelocity2.getVz(), 0.0);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var positionAndVelocity1 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final var positionAndVelocity2 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final var positionAndVelocity3 = new ECEFPositionAndVelocity();

        // check
        assertEquals(positionAndVelocity1.hashCode(), positionAndVelocity2.hashCode());
        assertNotEquals(positionAndVelocity1.hashCode(), positionAndVelocity3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var positionAndVelocity1 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final var positionAndVelocity2 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final var positionAndVelocity3 = new ECEFPositionAndVelocity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(positionAndVelocity1.equals((Object)positionAndVelocity1));
        //noinspection EqualsWithItself
        assertTrue(positionAndVelocity1.equals(positionAndVelocity1));
        assertTrue(positionAndVelocity1.equals(positionAndVelocity2));
        assertFalse(positionAndVelocity1.equals(positionAndVelocity3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(positionAndVelocity1.equals((Object)null));
        assertFalse(positionAndVelocity1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), positionAndVelocity1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var positionAndVelocity1 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final var positionAndVelocity2 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        final var positionAndVelocity3 = new ECEFPositionAndVelocity();

        assertTrue(positionAndVelocity1.equals(positionAndVelocity1, THRESHOLD));
        assertTrue(positionAndVelocity1.equals(positionAndVelocity2, THRESHOLD));
        assertFalse(positionAndVelocity1.equals(positionAndVelocity3, THRESHOLD));
        assertFalse(positionAndVelocity1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var positionAndVelocity1 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);

        final var positionAndVelocity2 = positionAndVelocity1.clone();

        assertEquals(positionAndVelocity1, positionAndVelocity2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final var z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var positionAndVelocity1 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(positionAndVelocity1);
        final var positionAndVelocity2 = SerializationHelper.<ECEFPositionAndVelocity>deserialize(bytes);

        // check
        assertEquals(positionAndVelocity1, positionAndVelocity2);
        assertNotSame(positionAndVelocity1, positionAndVelocity2);
    }
}
