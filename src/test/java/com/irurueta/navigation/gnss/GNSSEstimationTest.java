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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
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

class GNSSEstimationTest {

    private static final double MIN_POS_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POS_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SPEED_VALUE = -2.0;
    private static final double MAX_SPEED_VALUE = 2.0;

    private static final double MIN_CLOCK_OFFSET = -1e-5;
    private static final double MAX_CLOCK_OFFSET = 1e-5;

    private static final double MIN_CLOCK_DRIFT = -1e-6;
    private static final double MAX_CLOCK_DRIFT = 1e-6;

    private static final double THRESHOLD = 1e-8;

    @Test
    void testConstructor() {
        // test empty constructor
        var estimation = new GNSSEstimation();

        // check default values
        assertEquals(0.0, estimation.getX(), 0.0);
        assertEquals(0.0, estimation.getY(), 0.0);
        assertEquals(0.0, estimation.getZ(), 0.0);
        assertEquals(0.0, estimation.getVx(), 0.0);
        assertEquals(0.0, estimation.getVy(), 0.0);
        assertEquals(0.0, estimation.getVz(), 0.0);
        assertEquals(0.0, estimation.getClockOffset(), 0.0);
        assertEquals(0.0, estimation.getClockDrift(), 0.0);

        // test constructor with values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        // check default values
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // test constructor with measurement values
        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final var clockOffsetDistance = new Distance(clockOffset, DistanceUnit.METER);
        final var clockDriftSpeed = new Speed(clockDrift, SpeedUnit.METERS_PER_SECOND);

        estimation = new GNSSEstimation(distanceX, distanceY, distanceZ, speedX, speedY, speedZ, clockOffsetDistance,
                clockDriftSpeed);

        // check default values
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // test constructor with values and position
        final var position = new InhomogeneousPoint3D(x, y, z);
        estimation = new GNSSEstimation(position, vx, vy, vz, clockOffset, clockDrift);

        // check default values
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // test constructor with measurement values and position
        estimation = new GNSSEstimation(position, speedX, speedY, speedZ, clockOffsetDistance, clockDriftSpeed);

        // check default values
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // test constructor with ECEF position, ECEF velocity and clock offset and drift
        final var ecefPosition = new ECEFPosition(x, y, z);
        final var ecefVelocity = new ECEFVelocity(vx, vy, vz);
        estimation = new GNSSEstimation(ecefPosition, ecefVelocity, clockOffset, clockDrift);

        // check default values
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // test constructor with ECEF position, ECEF velocity and clock offset and drift measurements
        estimation = new GNSSEstimation(ecefPosition, ecefVelocity, clockOffsetDistance, clockDriftSpeed);

        // check default values
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // test constructor with ECEF position and velocity, and clock offset and drift
        final var positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition, ecefVelocity);
        estimation = new GNSSEstimation(positionAndVelocity, clockOffset, clockDrift);

        // check default values
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // test constructor with ECEF position and velocity, and clock offset and drift measurements
        estimation = new GNSSEstimation(positionAndVelocity, clockOffsetDistance, clockDriftSpeed);

        // check default values
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // test copy constructor
        final var estimation2 = new GNSSEstimation(estimation);

        // check default values
        assertEquals(x, estimation2.getX(), 0.0);
        assertEquals(y, estimation2.getY(), 0.0);
        assertEquals(z, estimation2.getZ(), 0.0);
        assertEquals(vx, estimation2.getVx(), 0.0);
        assertEquals(vy, estimation2.getVy(), 0.0);
        assertEquals(vz, estimation2.getVz(), 0.0);
        assertEquals(clockOffset, estimation2.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation2.getClockDrift(), 0.0);
    }

    @Test
    void testGetSetX() {
        final var estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        estimation.setX(x);

        // check
        assertEquals(x, estimation.getX(), 0.0);
    }

    @Test
    void testGetSetY() {
        final var estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        estimation.setY(y);

        // check
        assertEquals(y, estimation.getY(), 0.0);
    }

    @Test
    void testGetSetZ() {
        final var estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        estimation.setZ(z);

        // check
        assertEquals(z, estimation.getZ(), 0.0);
    }

    @Test
    void testSetPositionCoordinates() {
        final var estimation = new GNSSEstimation();

        // check default values
        assertEquals(0.0, estimation.getX(), 0.0);
        assertEquals(0.0, estimation.getY(), 0.0);
        assertEquals(0.0, estimation.getZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        estimation.setPositionCoordinates(x, y, z);

        // check
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
    }

    @Test
    void testGetSetVx() {
        final var estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getVx(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        estimation.setVx(vx);

        // check
        assertEquals(vx, estimation.getVx(), 0.0);
    }

    @Test
    void testGetSetVy() {
        final var estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getVy(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        estimation.setVy(vy);

        // check
        assertEquals(vy, estimation.getVy(), 0.0);
    }

    @Test
    void testGetSetVz() {
        final var estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getVz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        estimation.setVz(vz);

        // check
        assertEquals(vz, estimation.getVz(), 0.0);
    }

    @Test
    void testSetVelocityCoordinates() {
        final var estimation = new GNSSEstimation();

        // check default values
        assertEquals(0.0, estimation.getVx(), 0.0);
        assertEquals(0.0, estimation.getVy(), 0.0);
        assertEquals(0.0, estimation.getVz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        estimation.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
    }

    @Test
    void testGetSetClockOffset() {
        final var estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getClockOffset(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);

        estimation.setClockOffset(clockOffset);

        // check
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
    }

    @Test
    void testGetSetClockDrift() {
        final var estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getClockDrift(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        estimation.setClockDrift(clockDrift);

        // check
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);
    }

    @Test
    void testGetSetDistanceX() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var distanceX1 = estimation.getDistanceX();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var distanceX2 = new Distance(x, DistanceUnit.METER);

        estimation.setDistanceX(distanceX2);

        // check
        final var distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        estimation.getDistanceX(distanceX3);
        final var distanceX4 = estimation.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    void testGetSetDistanceY() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var distanceY1 = estimation.getDistanceY();

        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var distanceY2 = new Distance(y, DistanceUnit.METER);

        estimation.setDistanceY(distanceY2);

        // check
        final var distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        estimation.getDistanceY(distanceY3);
        final var distanceY4 = estimation.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    void testGetSetDistanceZ() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var distanceZ1 = estimation.getDistanceZ();

        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var distanceZ2 = new Distance(z, DistanceUnit.METER);

        estimation.setDistanceZ(distanceZ2);

        // check
        final var distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        estimation.getDistanceZ(distanceZ3);
        final var distanceZ4 = estimation.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    void testSetPositionCoordinatesWithDistances() {
        final var estimation = new GNSSEstimation();

        // check default values
        final var distanceX1 = estimation.getDistanceX();
        final var distanceY1 = estimation.getDistanceY();
        final var distanceZ1 = estimation.getDistanceZ();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());
        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());
        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var distanceX2 = new Distance(x, DistanceUnit.METER);
        final var distanceY2 = new Distance(y, DistanceUnit.METER);
        final var distanceZ2 = new Distance(z, DistanceUnit.METER);

        estimation.setPositionCoordinates(distanceX2, distanceY2, distanceZ2);

        // check
        final var distanceX3 = estimation.getDistanceX();
        final var distanceY3 = estimation.getDistanceY();
        final var distanceZ3 = estimation.getDistanceZ();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceZ2, distanceZ3);
    }

    @Test
    void testGetSetSpeedX() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var speedX1 = estimation.getSpeedX();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);

        estimation.setSpeedX(speedX2);

        // check
        final var speedX3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimation.getSpeedX(speedX3);
        final var speedX4 = estimation.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    void testGetSetSpeedY() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var speedY1 = estimation.getSpeedY();

        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);

        estimation.setSpeedY(speedY2);

        // check
        final var speedY3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimation.getSpeedY(speedY3);
        final var speedY4 = estimation.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    void testGetSetSpeedZ() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var speedZ1 = estimation.getSpeedZ();

        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        estimation.setSpeedZ(speedZ2);

        // check
        final var speedZ3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimation.getSpeedZ(speedZ3);
        final var speedZ4 = estimation.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    void testSetVelocityCoordinatesWithSpeed() {
        final var estimation = new GNSSEstimation();

        // check default values
        final var speedX1 = estimation.getSpeedX();
        final var speedY1 = estimation.getSpeedY();
        final var speedZ1 = estimation.getSpeedZ();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());
        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());
        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        estimation.setVelocityCoordinates(speedX2, speedY2, speedZ2);

        // check
        final var speedX3 = estimation.getSpeedX();
        final var speedY3 = estimation.getSpeedY();
        final var speedZ3 = estimation.getSpeedZ();

        assertEquals(speedX2, speedX3);
        assertEquals(speedY2, speedY3);
        assertEquals(speedZ2, speedZ3);
    }

    @Test
    void testGetSetClockOffsetDistance() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var clockOffset1 = estimation.getClockOffsetDistance();

        assertEquals(0.0, clockOffset1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, clockOffset1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);

        final var clockOffset2 = new Distance(clockOffset, DistanceUnit.METER);

        estimation.setClockOffset(clockOffset2);

        // check
        final var clockOffset3 = new Distance(0.0, DistanceUnit.KILOMETER);
        estimation.getClockOffsetDistance(clockOffset3);
        final var clockOffset4 = estimation.getClockOffsetDistance();

        assertEquals(clockOffset2, clockOffset3);
        assertEquals(clockOffset2, clockOffset4);
    }

    @Test
    void testGetSetClockDriftSpeed() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var clockDrift1 = estimation.getClockDriftSpeed();

        assertEquals(0.0, clockDrift1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, clockDrift1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var clockDrift2 = new Speed(clockDrift, SpeedUnit.METERS_PER_SECOND);

        estimation.setClockDrift(clockDrift2);

        // check
        final var clockDrift3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimation.getClockDriftSpeed(clockDrift3);
        final var clockDrift4 = estimation.getClockDriftSpeed();

        assertEquals(clockDrift2, clockDrift3);
        assertEquals(clockDrift2, clockDrift4);
    }

    @Test
    void testGetSetPosition() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var position1 = estimation.getPosition();

        assertEquals(position1, Point3D.create());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var position2 = new InhomogeneousPoint3D(x, y, z);

        estimation.setPosition(position2);

        // check
        final var position3 = Point3D.create();
        estimation.getPosition(position3);
        final var position4 = estimation.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetEcefPosition() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var position1 = estimation.getEcefPosition();

        assertEquals(0.0, position1.getX(), 0.0);
        assertEquals(0.0, position1.getY(), 0.0);
        assertEquals(0.0, position1.getZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var position2 = new ECEFPosition(x, y, z);
        estimation.setEcefPosition(position2);

        // check
        final var position3 = new ECEFPosition();
        estimation.getEcefPosition(position3);
        final var position4 = estimation.getEcefPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetEcefVelocity() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var velocity1 = estimation.getEcefVelocity();

        assertEquals(0.0, velocity1.getVx(), 0.0);
        assertEquals(0.0, velocity1.getVy(), 0.0);
        assertEquals(0.0, velocity1.getVz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var velocity2 = new ECEFVelocity(vx, vy, vz);
        estimation.setEcefVelocity(velocity2);

        // check
        final var velocity3 = new ECEFVelocity();
        estimation.getEcefVelocity(velocity3);
        final var velocity4 = estimation.getEcefVelocity();

        assertEquals(velocity2, velocity3);
        assertEquals(velocity2, velocity4);
    }

    @Test
    void testGetSetPositionAndVelocity() {
        final var estimation = new GNSSEstimation();

        // check default value
        final var positionAndVelocity1 = estimation.getPositionAndVelocity();

        assertEquals(0.0, positionAndVelocity1.getX(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getY(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getZ(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getVz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var positionAndVelocity2 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        estimation.setPositionAndVelocity(positionAndVelocity2);

        // check
        final var positionAndVelocity3 = new ECEFPositionAndVelocity();
        estimation.getPositionAndVelocity(positionAndVelocity3);
        final var positionAndVelocity4 = estimation.getPositionAndVelocity();

        assertEquals(positionAndVelocity2, positionAndVelocity3);
        assertEquals(positionAndVelocity2, positionAndVelocity4);
    }

    @Test
    void testAsArray() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final var result1 = new double[GNSSEstimation.NUM_PARAMETERS];
        estimation.asArray(result1);
        final var result2 = estimation.asArray();

        // check
        assertEquals(x, result1[0], 0.0);
        assertEquals(y, result1[1], 0.0);
        assertEquals(z, result1[2], 0.0);
        assertEquals(vx, result1[3], 0.0);
        assertEquals(vy, result1[4], 0.0);
        assertEquals(vz, result1[5], 0.0);
        assertEquals(clockOffset, result1[6], 0.0);
        assertEquals(clockDrift, result1[7], 0.0);
        assertArrayEquals(result1, result2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimation.asArray(new double[1]));
    }

    @Test
    void testFromArray() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var array = new double[]{x, y, z, vx, vy, vz, clockOffset, clockDrift};

        final var estimation = new GNSSEstimation();
        estimation.fromArray(array);

        // check
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimation.fromArray(new double[1]));
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final var result1 = new Matrix(GNSSEstimation.NUM_PARAMETERS, 1);
        estimation.asMatrix(result1);
        final var result2 = new Matrix(1, 1);
        estimation.asMatrix(result2);
        final var result3 = estimation.asMatrix();

        // check
        assertEquals(GNSSEstimation.NUM_PARAMETERS, result1.getRows());
        assertEquals(1, result1.getColumns());
        assertEquals(x, result1.getElementAtIndex(0), 0.0);
        assertEquals(y, result1.getElementAtIndex(1), 0.0);
        assertEquals(z, result1.getElementAtIndex(2), 0.0);
        assertEquals(vx, result1.getElementAtIndex(3), 0.0);
        assertEquals(vy, result1.getElementAtIndex(4), 0.0);
        assertEquals(vz, result1.getElementAtIndex(5), 0.0);
        assertEquals(clockOffset, result1.getElementAtIndex(6), 0.0);
        assertEquals(clockDrift, result1.getElementAtIndex(7), 0.0);
        assertEquals(result1, result2);
        assertEquals(result1, result3);
    }

    @Test
    void testFromMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var array = new double[]{x, y, z, vx, vy, vz, clockOffset, clockDrift};
        final var matrix = Matrix.newFromArray(array);

        final var estimation = new GNSSEstimation();
        estimation.fromMatrix(matrix);

        // check
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> estimation.fromMatrix(new Matrix(1, GNSSEstimation.NUM_PARAMETERS)));
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final var estimation2 = new GNSSEstimation();
        estimation1.copyTo(estimation2);

        // check
        assertEquals(x, estimation2.getX(), 0.0);
        assertEquals(y, estimation2.getY(), 0.0);
        assertEquals(z, estimation2.getZ(), 0.0);
        assertEquals(vx, estimation2.getVx(), 0.0);
        assertEquals(vy, estimation2.getVy(), 0.0);
        assertEquals(vz, estimation2.getVz(), 0.0);
        assertEquals(clockOffset, estimation2.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation2.getClockDrift(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final var estimation2 = new GNSSEstimation();
        estimation2.copyFrom(estimation1);

        // check
        assertEquals(x, estimation2.getX(), 0.0);
        assertEquals(y, estimation2.getY(), 0.0);
        assertEquals(z, estimation2.getZ(), 0.0);
        assertEquals(vx, estimation2.getVx(), 0.0);
        assertEquals(vy, estimation2.getVy(), 0.0);
        assertEquals(vz, estimation2.getVz(), 0.0);
        assertEquals(clockOffset, estimation2.getClockOffset(), 0.0);
        assertEquals(clockDrift, estimation2.getClockDrift(), 0.0);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var estimation2 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var estimation3 = new GNSSEstimation();

        assertEquals(estimation1.hashCode(), estimation2.hashCode());
        assertNotEquals(estimation1.hashCode(), estimation3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var estimation2 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var estimation3 = new GNSSEstimation();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(estimation1.equals((Object) estimation1));
        //noinspection EqualsWithItself
        assertTrue(estimation1.equals(estimation1));
        assertTrue(estimation1.equals(estimation2));
        assertFalse(estimation1.equals(estimation3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(estimation1.equals((Object) null));
        assertFalse(estimation1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), estimation1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var estimation2 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final var estimation3 = new GNSSEstimation();

        assertTrue(estimation1.equals(estimation1, THRESHOLD));
        assertTrue(estimation1.equals(estimation2, THRESHOLD));
        assertFalse(estimation1.equals(estimation3, THRESHOLD));
        assertFalse(estimation1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final var estimation2 = estimation1.clone();

        assertEquals(estimation1, estimation2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final var clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final var estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(estimation1);
        final var estimation2 = SerializationHelper.<GNSSEstimation>deserialize(bytes);

        // check
        assertEquals(estimation1, estimation2);
        assertNotSame(estimation1, estimation2);
    }
}
