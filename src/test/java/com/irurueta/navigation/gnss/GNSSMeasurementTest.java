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
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class GNSSMeasurementTest {

    private static final double MIN_POS_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POS_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SPEED_VALUE = -2.0;
    private static final double MAX_SPEED_VALUE = 2.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        GNSSMeasurement measurement = new GNSSMeasurement();

        // check default values
        assertEquals(measurement.getPseudoRange(), 0.0, 0.0);
        assertEquals(measurement.getPseudoRate(), 0.0, 0.0);
        assertEquals(measurement.getX(), 0.0, 0.0);
        assertEquals(measurement.getY(), 0.0, 0.0);
        assertEquals(measurement.getZ(), 0.0, 0.0);
        assertEquals(measurement.getVx(), 0.0, 0.0);
        assertEquals(measurement.getVy(), 0.0, 0.0);
        assertEquals(measurement.getVz(), 0.0, 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        measurement = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z,
                vx, vy, vz);

        // check default values
        assertEquals(measurement.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement.getX(), x, 0.0);
        assertEquals(measurement.getY(), y, 0.0);
        assertEquals(measurement.getZ(), z, 0.0);
        assertEquals(measurement.getVx(), vx, 0.0);
        assertEquals(measurement.getVy(), vy, 0.0);
        assertEquals(measurement.getVz(), vz, 0.0);


        // test constructor with measurement values
        final Distance pseudoRangeDistance = new Distance(pseudoRange, DistanceUnit.METER);
        final Speed pseudoRateSpeed = new Speed(pseudoRate, SpeedUnit.METERS_PER_SECOND);
        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);
        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        measurement = new GNSSMeasurement(pseudoRangeDistance, pseudoRateSpeed,
                distanceX, distanceY, distanceZ, speedX, speedY, speedZ);

        // check default values
        assertEquals(measurement.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement.getX(), x, 0.0);
        assertEquals(measurement.getY(), y, 0.0);
        assertEquals(measurement.getZ(), z, 0.0);
        assertEquals(measurement.getVx(), vx, 0.0);
        assertEquals(measurement.getVy(), vy, 0.0);
        assertEquals(measurement.getVz(), vz, 0.0);


        // test constructor with measurement values and position
        final Point3D position = new InhomogeneousPoint3D(x, y, z);

        measurement = new GNSSMeasurement(pseudoRangeDistance, pseudoRateSpeed,
                position, speedX, speedY, speedZ);

        // check default values
        assertEquals(measurement.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement.getX(), x, 0.0);
        assertEquals(measurement.getY(), y, 0.0);
        assertEquals(measurement.getZ(), z, 0.0);
        assertEquals(measurement.getVx(), vx, 0.0);
        assertEquals(measurement.getVy(), vy, 0.0);
        assertEquals(measurement.getVz(), vz, 0.0);


        // test constructor with ECEF position and velocity
        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        final ECEFVelocity ecefVelocity = new ECEFVelocity(vx, vy, vz);
        measurement = new GNSSMeasurement(pseudoRange, pseudoRate,
                ecefPosition, ecefVelocity);

        // check default values
        assertEquals(measurement.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement.getX(), x, 0.0);
        assertEquals(measurement.getY(), y, 0.0);
        assertEquals(measurement.getZ(), z, 0.0);
        assertEquals(measurement.getVx(), vx, 0.0);
        assertEquals(measurement.getVy(), vy, 0.0);
        assertEquals(measurement.getVz(), vz, 0.0);


        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity(ecefPosition, ecefVelocity);
        measurement = new GNSSMeasurement(pseudoRange, pseudoRate, positionAndVelocity);

        // check default values
        assertEquals(measurement.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement.getX(), x, 0.0);
        assertEquals(measurement.getY(), y, 0.0);
        assertEquals(measurement.getZ(), z, 0.0);
        assertEquals(measurement.getVx(), vx, 0.0);
        assertEquals(measurement.getVy(), vy, 0.0);
        assertEquals(measurement.getVz(), vz, 0.0);


        measurement = new GNSSMeasurement(pseudoRangeDistance, pseudoRateSpeed,
                ecefPosition, ecefVelocity);

        // check default values
        assertEquals(measurement.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement.getX(), x, 0.0);
        assertEquals(measurement.getY(), y, 0.0);
        assertEquals(measurement.getZ(), z, 0.0);
        assertEquals(measurement.getVx(), vx, 0.0);
        assertEquals(measurement.getVy(), vy, 0.0);
        assertEquals(measurement.getVz(), vz, 0.0);


        measurement = new GNSSMeasurement(pseudoRangeDistance, pseudoRateSpeed,
                positionAndVelocity);

        // check default values
        assertEquals(measurement.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement.getX(), x, 0.0);
        assertEquals(measurement.getY(), y, 0.0);
        assertEquals(measurement.getZ(), z, 0.0);
        assertEquals(measurement.getVx(), vx, 0.0);
        assertEquals(measurement.getVy(), vy, 0.0);
        assertEquals(measurement.getVz(), vz, 0.0);


        // test copy constructor
        final GNSSMeasurement measurement2 = new GNSSMeasurement(measurement);

        // check default values
        assertEquals(measurement2.getPseudoRange(), measurement.getPseudoRange(), 0.0);
        assertEquals(measurement2.getPseudoRate(), measurement.getPseudoRate(), 0.0);
        assertEquals(measurement2.getX(), measurement.getX(), 0.0);
        assertEquals(measurement2.getY(), measurement.getY(), 0.0);
        assertEquals(measurement2.getZ(), measurement.getZ(), 0.0);
        assertEquals(measurement2.getVx(), measurement.getVx(), 0.0);
        assertEquals(measurement2.getVy(), measurement.getVy(), 0.0);
        assertEquals(measurement2.getVz(), measurement.getVz(), 0.0);
    }

    @Test
    public void testGetSetPseudoRange() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        assertEquals(measurement.getPseudoRange(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        measurement.setPseudoRange(pseudoRange);

        // check
        assertEquals(measurement.getPseudoRange(), pseudoRange, 0.0);
    }

    @Test
    public void testGetSetPseudoRangeDistance() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Distance distance1 = measurement.getPseudoRangeDistance();

        assertEquals(distance1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distance1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distance2 = new Distance(pseudoRange, DistanceUnit.METER);
        measurement.setPseudoRangeDistance(distance2);

        // check
        final Distance distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        measurement.getPseudoRangeDistance(distance3);
        final Distance distance4 = measurement.getPseudoRangeDistance();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    public void testGetSetPseudoRate() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        assertEquals(measurement.getPseudoRate(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        measurement.setPseudoRate(pseudoRate);

        // check
        assertEquals(measurement.getPseudoRate(), pseudoRate, 0.0);
    }

    @Test
    public void testGetSetPseudoRateSpeed() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Speed speed1 = measurement.getPseudoRateSpeed();

        assertEquals(speed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speed1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speed2 = new Speed(pseudoRate, SpeedUnit.METERS_PER_SECOND);
        measurement.setPseudoRateSpeed(speed2);

        // check
        final Speed speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        measurement.getPseudoRateSpeed(speed3);
        final Speed speed4 = measurement.getPseudoRateSpeed();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    public void testGetSetX() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        assertEquals(measurement.getX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        measurement.setX(x);

        // check
        assertEquals(measurement.getX(), x, 0.0);
    }

    @Test
    public void testGetSetDistanceX() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Distance distanceX1 = measurement.getDistanceX();

        assertEquals(distanceX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceX1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);
        measurement.setDistanceX(distanceX2);

        // check
        final Distance distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        measurement.getDistanceX(distanceX3);
        final Distance distanceX4 = measurement.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    public void testGetSetY() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        assertEquals(measurement.getY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        measurement.setY(y);

        // check
        assertEquals(measurement.getY(), y, 0.0);
    }

    @Test
    public void testGetSetDistanceY() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Distance distanceY1 = measurement.getDistanceY();

        assertEquals(distanceY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceY1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);
        measurement.setDistanceY(distanceY2);

        // check
        final Distance distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        measurement.getDistanceY(distanceY3);
        final Distance distanceY4 = measurement.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    public void testGetSetZ() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        assertEquals(measurement.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        measurement.setZ(z);

        // check
        assertEquals(measurement.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetDistanceZ() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Distance distanceZ1 = measurement.getDistanceZ();

        assertEquals(distanceZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceZ1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);
        measurement.setDistanceZ(distanceZ2);

        // check
        final Distance distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        measurement.getDistanceZ(distanceZ3);
        final Distance distanceZ4 = measurement.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    public void testSetPositionCoordinates() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default values
        assertEquals(measurement.getX(), 0.0, 0.0);
        assertEquals(measurement.getY(), 0.0, 0.0);
        assertEquals(measurement.getZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        measurement.setPositionCoordinates(x, y, z);

        // check
        assertEquals(measurement.getX(), x, 0.0);
        assertEquals(measurement.getY(), y, 0.0);
        assertEquals(measurement.getZ(), z, 0.0);
    }

    @Test
    public void setPositionCoordinatesWithDistances() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default values
        final Distance distanceX1 = measurement.getDistanceX();
        final Distance distanceY1 = measurement.getDistanceY();
        final Distance distanceZ1 = measurement.getDistanceZ();

        assertEquals(distanceX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceX1.getUnit(), DistanceUnit.METER);
        assertEquals(distanceY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceY1.getUnit(), DistanceUnit.METER);
        assertEquals(distanceZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceZ1.getUnit(), DistanceUnit.METER);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);
        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);
        measurement.setPositionCoordinates(distanceX2, distanceY2, distanceZ2);

        // check
        final Distance distanceX3 = measurement.getDistanceX();
        final Distance distanceY3 = measurement.getDistanceY();
        final Distance distanceZ3 = measurement.getDistanceZ();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceZ2, distanceZ3);
    }

    @Test
    public void testGetSetPosition() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Point3D position1 = measurement.getPosition();

        // check
        assertEquals(position1, Point3D.create());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final Point3D position2 = new InhomogeneousPoint3D(x, y, z);
        measurement.setPosition(position2);

        // check
        final Point3D position3 = measurement.getPosition();
        final Point3D position4 = Point3D.create();
        measurement.getPosition(position4);

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetEcefPosition() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final ECEFPosition position1 = measurement.getEcefPosition();

        // check
        assertEquals(position1.getX(), 0.0, 0.0);
        assertEquals(position1.getY(), 0.0, 0.0);
        assertEquals(position1.getZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final ECEFPosition position2 = new ECEFPosition(x, y, z);
        measurement.setEcefPosition(position2);

        // check
        final ECEFPosition position3 = measurement.getEcefPosition();
        final ECEFPosition position4 = new ECEFPosition();
        measurement.getEcefPosition(position4);

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetVx() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        assertEquals(measurement.getVx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        measurement.setVx(vx);

        // check
        assertEquals(measurement.getVx(), vx, 0.0);
    }

    @Test
    public void testGetSetSpeedX() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Speed speedX1 = measurement.getSpeedX();

        assertEquals(speedX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedX1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        measurement.setSpeedX(speedX2);

        // check
        final Speed speedX3 = measurement.getSpeedX();
        final Speed speedX4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        measurement.getSpeedX(speedX4);

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    public void testGetSetVy() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        assertEquals(measurement.getVy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        measurement.setVy(vy);

        // check
        assertEquals(measurement.getVy(), vy, 0.0);
    }

    @Test
    public void testGetSetSpeedY() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Speed speedY1 = measurement.getSpeedY();

        assertEquals(speedY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedY1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        measurement.setSpeedY(speedY2);

        // check
        final Speed speedY3 = measurement.getSpeedY();
        final Speed speedY4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        measurement.getSpeedY(speedY4);

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    public void testGetSetVz() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        assertEquals(measurement.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        measurement.setVz(vz);

        // check
        assertEquals(measurement.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetSpeedZ() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final Speed speedZ1 = measurement.getSpeedZ();

        assertEquals(speedZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedZ1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        measurement.setSpeedZ(speedZ2);

        // check
        final Speed speedZ3 = measurement.getSpeedZ();
        final Speed speedZ4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        measurement.getSpeedZ(speedZ4);

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    public void testSetVelocityCoordinates() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default values
        assertEquals(measurement.getVx(), 0.0, 0.0);
        assertEquals(measurement.getVy(), 0.0, 0.0);
        assertEquals(measurement.getVz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        measurement.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(measurement.getVx(), vx, 0.0);
        assertEquals(measurement.getVy(), vy, 0.0);
        assertEquals(measurement.getVz(), vz, 0.0);
    }

    @Test
    public void testSetVelocityCoordinatesWithSpeed() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default values
        final Speed speedX1 = measurement.getSpeedX();
        final Speed speedY1 = measurement.getSpeedY();
        final Speed speedZ1 = measurement.getSpeedZ();

        assertEquals(speedX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedX1.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedY1.getUnit(), SpeedUnit.METERS_PER_SECOND);
        assertEquals(speedZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speedZ1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        measurement.setVelocityCoordinates(speedX2, speedY2, speedZ2);

        // check
        final Speed speedX3 = measurement.getSpeedX();
        final Speed speedY3 = measurement.getSpeedY();
        final Speed speedZ3 = measurement.getSpeedZ();

        assertEquals(speedX2, speedX3);
        assertEquals(speedY2, speedY3);
        assertEquals(speedZ2, speedZ3);
    }

    @Test
    public void testGetSetEcefVelocity() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final ECEFVelocity velocity1 = measurement.getEcefVelocity();

        assertEquals(velocity1.getVx(), 0.0, 0.0);
        assertEquals(velocity1.getVy(), 0.0, 0.0);
        assertEquals(velocity1.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFVelocity velocity2 = new ECEFVelocity(vx, vy, vz);

        measurement.setEcefVelocity(velocity2);

        // check
        final ECEFVelocity velocity3 = new ECEFVelocity();
        measurement.getEcefVelocity(velocity3);
        final ECEFVelocity velocity4 = measurement.getEcefVelocity();

        assertEquals(velocity2, velocity3);
        assertEquals(velocity2, velocity4);
    }

    @Test
    public void testGetSetPositionAndVelocity() {
        final GNSSMeasurement measurement = new GNSSMeasurement();

        // check default value
        final ECEFPositionAndVelocity positionAndVelocity1 = measurement.getPositionAndVelocity();

        assertEquals(positionAndVelocity1.getX(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getY(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getZ(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getVx(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getVy(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final ECEFPositionAndVelocity positionAndVelocity2 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);

        measurement.setPositionAndVelocity(positionAndVelocity2);

        // check
        final ECEFPositionAndVelocity positionAndVelocity3 = new ECEFPositionAndVelocity();
        measurement.getPositionAndVelocity(positionAndVelocity3);
        final ECEFPositionAndVelocity positionAndVelocity4 = measurement.getPositionAndVelocity();

        assertEquals(positionAndVelocity2, positionAndVelocity3);
        assertEquals(positionAndVelocity2, positionAndVelocity4);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final GNSSMeasurement measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);

        final GNSSMeasurement measurement2 = new GNSSMeasurement();
        measurement1.copyTo(measurement2);

        // check
        assertEquals(measurement2.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement2.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement2.getX(), x, 0.0);
        assertEquals(measurement2.getY(), y, 0.0);
        assertEquals(measurement2.getZ(), z, 0.0);
        assertEquals(measurement2.getVx(), vx, 0.0);
        assertEquals(measurement2.getVy(), vy, 0.0);
        assertEquals(measurement2.getVz(), vz, 0.0);
        assertEquals(measurement1, measurement2);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final GNSSMeasurement measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);

        final GNSSMeasurement measurement2 = new GNSSMeasurement();
        measurement2.copyFrom(measurement1);

        // check
        assertEquals(measurement2.getPseudoRange(), pseudoRange, 0.0);
        assertEquals(measurement2.getPseudoRate(), pseudoRate, 0.0);
        assertEquals(measurement2.getX(), x, 0.0);
        assertEquals(measurement2.getY(), y, 0.0);
        assertEquals(measurement2.getZ(), z, 0.0);
        assertEquals(measurement2.getVx(), vx, 0.0);
        assertEquals(measurement2.getVy(), vy, 0.0);
        assertEquals(measurement2.getVz(), vz, 0.0);
        assertEquals(measurement1, measurement2);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final GNSSMeasurement measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);
        final GNSSMeasurement measurement2 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);
        final GNSSMeasurement measurement3 = new GNSSMeasurement();

        assertEquals(measurement1.hashCode(), measurement2.hashCode());
        assertNotEquals(measurement1.hashCode(), measurement3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final GNSSMeasurement measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);
        final GNSSMeasurement measurement2 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);
        final GNSSMeasurement measurement3 = new GNSSMeasurement();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(measurement1.equals((Object)measurement1));
        //noinspection EqualsWithItself
        assertTrue(measurement1.equals(measurement1));
        assertTrue(measurement1.equals(measurement2));
        assertFalse(measurement1.equals(measurement3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(measurement1.equals((Object)null));
        assertFalse(measurement1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(measurement1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final GNSSMeasurement measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);
        final GNSSMeasurement measurement2 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);
        final GNSSMeasurement measurement3 = new GNSSMeasurement();

        assertTrue(measurement1.equals(measurement1, THRESHOLD));
        assertTrue(measurement1.equals(measurement2, THRESHOLD));
        assertFalse(measurement1.equals(measurement3, THRESHOLD));
        assertFalse(measurement1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final double x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final double z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final double vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final double vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final GNSSMeasurement measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate,
                x, y, z, vx, vy, vz);

        final Object measurement2 = measurement1.clone();

        assertEquals(measurement1, measurement2);
    }
}
