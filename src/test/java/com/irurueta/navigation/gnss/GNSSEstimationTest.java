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
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class GNSSEstimationTest {

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
    public void testConstructor() {
        // test empty constructor
        GNSSEstimation estimation = new GNSSEstimation();

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
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

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
        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Distance clockOffsetDistance = new Distance(clockOffset, DistanceUnit.METER);
        final Speed clockDriftSpeed = new Speed(clockDrift, SpeedUnit.METERS_PER_SECOND);

        estimation = new GNSSEstimation(distanceX, distanceY, distanceZ, speedX, speedY, speedZ,
                clockOffsetDistance, clockDriftSpeed);

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
        final Point3D position = new InhomogeneousPoint3D(x, y, z);
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
        estimation = new GNSSEstimation(position, speedX, speedY, speedZ,
                clockOffsetDistance, clockDriftSpeed);

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
        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        final ECEFVelocity ecefVelocity = new ECEFVelocity(vx, vy, vz);
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
        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity(ecefPosition, ecefVelocity);
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
        final GNSSEstimation estimation2 = new GNSSEstimation(estimation);

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
    public void testGetSetX() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getX(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        estimation.setX(x);

        // check
        assertEquals(x, estimation.getX(), 0.0);
    }

    @Test
    public void testGetSetY() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        estimation.setY(y);

        // check
        assertEquals(y, estimation.getY(), 0.0);
    }

    @Test
    public void testGetSetZ() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        estimation.setZ(z);

        // check
        assertEquals(z, estimation.getZ(), 0.0);
    }

    @Test
    public void testSetPositionCoordinates() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default values
        assertEquals(0.0, estimation.getX(), 0.0);
        assertEquals(0.0, estimation.getY(), 0.0);
        assertEquals(0.0, estimation.getZ(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        estimation.setPositionCoordinates(x, y, z);

        // check
        assertEquals(x, estimation.getX(), 0.0);
        assertEquals(y, estimation.getY(), 0.0);
        assertEquals(z, estimation.getZ(), 0.0);
    }

    @Test
    public void testGetSetVx() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getVx(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        estimation.setVx(vx);

        // check
        assertEquals(vx, estimation.getVx(), 0.0);
    }

    @Test
    public void testGetSetVy() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getVy(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        estimation.setVy(vy);

        // check
        assertEquals(vy, estimation.getVy(), 0.0);
    }

    @Test
    public void testGetSetVz() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getVz(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        estimation.setVz(vz);

        // check
        assertEquals(vz, estimation.getVz(), 0.0);
    }

    @Test
    public void testSetVelocityCoordinates() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default values
        assertEquals(0.0, estimation.getVx(), 0.0);
        assertEquals(0.0, estimation.getVy(), 0.0);
        assertEquals(0.0, estimation.getVz(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        estimation.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(vx, estimation.getVx(), 0.0);
        assertEquals(vy, estimation.getVy(), 0.0);
        assertEquals(vz, estimation.getVz(), 0.0);
    }

    @Test
    public void testGetSetClockOffset() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getClockOffset(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);

        estimation.setClockOffset(clockOffset);

        // check
        assertEquals(clockOffset, estimation.getClockOffset(), 0.0);
    }

    @Test
    public void testGetSetClockDrift() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        assertEquals(0.0, estimation.getClockDrift(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        estimation.setClockDrift(clockDrift);

        // check
        assertEquals(clockDrift, estimation.getClockDrift(), 0.0);
    }

    @Test
    public void testGetSetDistanceX() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Distance distanceX1 = estimation.getDistanceX();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);

        estimation.setDistanceX(distanceX2);

        // check
        final Distance distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        estimation.getDistanceX(distanceX3);
        final Distance distanceX4 = estimation.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    public void testGetSetDistanceY() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Distance distanceY1 = estimation.getDistanceY();

        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);

        estimation.setDistanceY(distanceY2);

        // check
        final Distance distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        estimation.getDistanceY(distanceY3);
        final Distance distanceY4 = estimation.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    public void testGetSetDistanceZ() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Distance distanceZ1 = estimation.getDistanceZ();

        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);

        estimation.setDistanceZ(distanceZ2);

        // check
        final Distance distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        estimation.getDistanceZ(distanceZ3);
        final Distance distanceZ4 = estimation.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    public void testSetPositionCoordinatesWithDistances() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default values
        final Distance distanceX1 = estimation.getDistanceX();
        final Distance distanceY1 = estimation.getDistanceY();
        final Distance distanceZ1 = estimation.getDistanceZ();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());
        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());
        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);
        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);

        estimation.setPositionCoordinates(distanceX2, distanceY2, distanceZ2);

        // check
        final Distance distanceX3 = estimation.getDistanceX();
        final Distance distanceY3 = estimation.getDistanceY();
        final Distance distanceZ3 = estimation.getDistanceZ();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceZ2, distanceZ3);
    }

    @Test
    public void testGetSetSpeedX() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Speed speedX1 = estimation.getSpeedX();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);

        estimation.setSpeedX(speedX2);

        // check
        final Speed speedX3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimation.getSpeedX(speedX3);
        final Speed speedX4 = estimation.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    public void testGetSetSpeedY() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Speed speedY1 = estimation.getSpeedY();

        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);

        estimation.setSpeedY(speedY2);

        // check
        final Speed speedY3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimation.getSpeedY(speedY3);
        final Speed speedY4 = estimation.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    public void testGetSetSpeedZ() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Speed speedZ1 = estimation.getSpeedZ();

        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        estimation.setSpeedZ(speedZ2);

        // check
        final Speed speedZ3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimation.getSpeedZ(speedZ3);
        final Speed speedZ4 = estimation.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    public void testSetVelocityCoordinatesWithSpeed() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default values
        final Speed speedX1 = estimation.getSpeedX();
        final Speed speedY1 = estimation.getSpeedY();
        final Speed speedZ1 = estimation.getSpeedZ();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());
        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());
        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        estimation.setVelocityCoordinates(speedX2, speedY2, speedZ2);

        // check
        final Speed speedX3 = estimation.getSpeedX();
        final Speed speedY3 = estimation.getSpeedY();
        final Speed speedZ3 = estimation.getSpeedZ();

        assertEquals(speedX2, speedX3);
        assertEquals(speedY2, speedY3);
        assertEquals(speedZ2, speedZ3);
    }

    @Test
    public void testGetSetClockOffsetDistance() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Distance clockOffset1 = estimation.getClockOffsetDistance();

        assertEquals(0.0, clockOffset1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, clockOffset1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);

        final Distance clockOffset2 = new Distance(clockOffset, DistanceUnit.METER);

        estimation.setClockOffset(clockOffset2);

        // check
        final Distance clockOffset3 = new Distance(0.0, DistanceUnit.KILOMETER);
        estimation.getClockOffsetDistance(clockOffset3);
        final Distance clockOffset4 = estimation.getClockOffsetDistance();

        assertEquals(clockOffset2, clockOffset3);
        assertEquals(clockOffset2, clockOffset4);
    }

    @Test
    public void testGetSetClockDriftSpeed() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Speed clockDrift1 = estimation.getClockDriftSpeed();

        assertEquals(0.0, clockDrift1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, clockDrift1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final Speed clockDrift2 = new Speed(clockDrift, SpeedUnit.METERS_PER_SECOND);

        estimation.setClockDrift(clockDrift2);

        // check
        final Speed clockDrift3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimation.getClockDriftSpeed(clockDrift3);
        final Speed clockDrift4 = estimation.getClockDriftSpeed();

        assertEquals(clockDrift2, clockDrift3);
        assertEquals(clockDrift2, clockDrift4);
    }

    @Test
    public void testGetSetPosition() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final Point3D position1 = estimation.getPosition();

        assertEquals(position1, Point3D.create());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Point3D position2 = new InhomogeneousPoint3D(x, y, z);

        estimation.setPosition(position2);

        // check
        final Point3D position3 = Point3D.create();
        estimation.getPosition(position3);
        final Point3D position4 = estimation.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetEcefPosition() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final ECEFPosition position1 = estimation.getEcefPosition();

        assertEquals(0.0, position1.getX(), 0.0);
        assertEquals(0.0, position1.getY(), 0.0);
        assertEquals(0.0, position1.getZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final ECEFPosition position2 = new ECEFPosition(x, y, z);
        estimation.setEcefPosition(position2);

        // check
        final ECEFPosition position3 = new ECEFPosition();
        estimation.getEcefPosition(position3);
        final ECEFPosition position4 = estimation.getEcefPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetEcefVelocity() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final ECEFVelocity velocity1 = estimation.getEcefVelocity();

        assertEquals(0.0, velocity1.getVx(), 0.0);
        assertEquals(0.0, velocity1.getVy(), 0.0);
        assertEquals(0.0, velocity1.getVz(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFVelocity velocity2 = new ECEFVelocity(vx, vy, vz);
        estimation.setEcefVelocity(velocity2);

        // check
        final ECEFVelocity velocity3 = new ECEFVelocity();
        estimation.getEcefVelocity(velocity3);
        final ECEFVelocity velocity4 = estimation.getEcefVelocity();

        assertEquals(velocity2, velocity3);
        assertEquals(velocity2, velocity4);
    }

    @Test
    public void testGetSetPositionAndVelocity() {
        final GNSSEstimation estimation = new GNSSEstimation();

        // check default value
        final ECEFPositionAndVelocity positionAndVelocity1 = estimation.getPositionAndVelocity();

        assertEquals(0.0, positionAndVelocity1.getX(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getY(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getZ(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getVx(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getVy(), 0.0);
        assertEquals(0.0, positionAndVelocity1.getVz(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final ECEFPositionAndVelocity positionAndVelocity2 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        estimation.setPositionAndVelocity(positionAndVelocity2);

        // check
        final ECEFPositionAndVelocity positionAndVelocity3 = new ECEFPositionAndVelocity();
        estimation.getPositionAndVelocity(positionAndVelocity3);
        final ECEFPositionAndVelocity positionAndVelocity4 = estimation.getPositionAndVelocity();

        assertEquals(positionAndVelocity2, positionAndVelocity3);
        assertEquals(positionAndVelocity2, positionAndVelocity4);
    }

    @Test
    public void testAsArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final double[] result1 = new double[GNSSEstimation.NUM_PARAMETERS];
        estimation.asArray(result1);
        final double[] result2 = estimation.asArray();

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
        try {
            estimation.asArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFromArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final double[] array = new double[]{x, y, z, vx, vy, vz, clockOffset, clockDrift};

        final GNSSEstimation estimation = new GNSSEstimation();
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
        try {
            estimation.fromArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final Matrix result1 = new Matrix(GNSSEstimation.NUM_PARAMETERS, 1);
        estimation.asMatrix(result1);
        final Matrix result2 = new Matrix(1, 1);
        estimation.asMatrix(result2);
        final Matrix result3 = estimation.asMatrix();

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
    public void testFromMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final double[] array = new double[]{x, y, z, vx, vy, vz, clockOffset, clockDrift};
        final Matrix matrix = Matrix.newFromArray(array);

        final GNSSEstimation estimation = new GNSSEstimation();
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
        try {
            estimation.fromMatrix(new Matrix(1, GNSSEstimation.NUM_PARAMETERS));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final GNSSEstimation estimation2 = new GNSSEstimation();
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
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final GNSSEstimation estimation2 = new GNSSEstimation();
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
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final GNSSEstimation estimation2 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final GNSSEstimation estimation3 = new GNSSEstimation();

        assertEquals(estimation1.hashCode(), estimation2.hashCode());
        assertNotEquals(estimation1.hashCode(), estimation3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final GNSSEstimation estimation2 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final GNSSEstimation estimation3 = new GNSSEstimation();

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
        assertFalse(estimation1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final GNSSEstimation estimation2 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);
        final GNSSEstimation estimation3 = new GNSSEstimation();

        assertTrue(estimation1.equals(estimation1, THRESHOLD));
        assertTrue(estimation1.equals(estimation2, THRESHOLD));
        assertFalse(estimation1.equals(estimation3, THRESHOLD));
        assertFalse(estimation1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET, MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT, MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        final Object estimation2 = estimation1.clone();

        assertEquals(estimation1, estimation2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double clockOffset = randomizer.nextDouble(MIN_CLOCK_OFFSET,
                MAX_CLOCK_OFFSET);
        final double clockDrift = randomizer.nextDouble(MIN_CLOCK_DRIFT,
                MAX_CLOCK_DRIFT);

        final GNSSEstimation estimation1 = new GNSSEstimation(x, y, z, vx, vy, vz, clockOffset, clockDrift);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(estimation1);
        final GNSSEstimation estimation2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(estimation1, estimation2);
        assertNotSame(estimation1, estimation2);
    }
}
