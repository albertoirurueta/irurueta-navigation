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

class GNSSMeasurementTest {

    private static final double MIN_POS_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POS_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SPEED_VALUE = -2.0;
    private static final double MAX_SPEED_VALUE = 2.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var measurement = new GNSSMeasurement();

        // check default values
        assertEquals(0.0, measurement.getPseudoRange(), 0.0);
        assertEquals(0.0, measurement.getPseudoRate(), 0.0);
        assertEquals(0.0, measurement.getX(), 0.0);
        assertEquals(0.0, measurement.getY(), 0.0);
        assertEquals(0.0, measurement.getZ(), 0.0);
        assertEquals(0.0, measurement.getVx(), 0.0);
        assertEquals(0.0, measurement.getVy(), 0.0);
        assertEquals(0.0, measurement.getVz(), 0.0);

        // test constructor with values
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        measurement = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);

        // check default values
        assertEquals(pseudoRange, measurement.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement.getPseudoRate(), 0.0);
        assertEquals(x, measurement.getX(), 0.0);
        assertEquals(y, measurement.getY(), 0.0);
        assertEquals(z, measurement.getZ(), 0.0);
        assertEquals(vx, measurement.getVx(), 0.0);
        assertEquals(vy, measurement.getVy(), 0.0);
        assertEquals(vz, measurement.getVz(), 0.0);


        // test constructor with measurement values
        final var pseudoRangeDistance = new Distance(pseudoRange, DistanceUnit.METER);
        final var pseudoRateSpeed = new Speed(pseudoRate, SpeedUnit.METERS_PER_SECOND);
        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);
        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        measurement = new GNSSMeasurement(pseudoRangeDistance, pseudoRateSpeed, distanceX, distanceY, distanceZ,
                speedX, speedY, speedZ);

        // check default values
        assertEquals(pseudoRange, measurement.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement.getPseudoRate(), 0.0);
        assertEquals(x, measurement.getX(), 0.0);
        assertEquals(y, measurement.getY(), 0.0);
        assertEquals(z, measurement.getZ(), 0.0);
        assertEquals(vx, measurement.getVx(), 0.0);
        assertEquals(vy, measurement.getVy(), 0.0);
        assertEquals(vz, measurement.getVz(), 0.0);

        // test constructor with measurement values and position
        final var position = new InhomogeneousPoint3D(x, y, z);

        measurement = new GNSSMeasurement(pseudoRangeDistance, pseudoRateSpeed, position, speedX, speedY, speedZ);

        // check default values
        assertEquals(pseudoRange, measurement.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement.getPseudoRate(), 0.0);
        assertEquals(x, measurement.getX(), 0.0);
        assertEquals(y, measurement.getY(), 0.0);
        assertEquals(z, measurement.getZ(), 0.0);
        assertEquals(vx, measurement.getVx(), 0.0);
        assertEquals(vy, measurement.getVy(), 0.0);
        assertEquals(vz, measurement.getVz(), 0.0);

        // test constructor with ECEF position and velocity
        final var ecefPosition = new ECEFPosition(x, y, z);
        final var ecefVelocity = new ECEFVelocity(vx, vy, vz);
        measurement = new GNSSMeasurement(pseudoRange, pseudoRate, ecefPosition, ecefVelocity);

        // check default values
        assertEquals(pseudoRange, measurement.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement.getPseudoRate(), 0.0);
        assertEquals(x, measurement.getX(), 0.0);
        assertEquals(y, measurement.getY(), 0.0);
        assertEquals(z, measurement.getZ(), 0.0);
        assertEquals(vx, measurement.getVx(), 0.0);
        assertEquals(vy, measurement.getVy(), 0.0);
        assertEquals(vz, measurement.getVz(), 0.0);


        final var positionAndVelocity = new ECEFPositionAndVelocity(ecefPosition, ecefVelocity);
        measurement = new GNSSMeasurement(pseudoRange, pseudoRate, positionAndVelocity);

        // check default values
        assertEquals(pseudoRange, measurement.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement.getPseudoRate(), 0.0);
        assertEquals(x, measurement.getX(), 0.0);
        assertEquals(y, measurement.getY(), 0.0);
        assertEquals(z, measurement.getZ(), 0.0);
        assertEquals(vx, measurement.getVx(), 0.0);
        assertEquals(vy, measurement.getVy(), 0.0);
        assertEquals(vz, measurement.getVz(), 0.0);

        measurement = new GNSSMeasurement(pseudoRangeDistance, pseudoRateSpeed, ecefPosition, ecefVelocity);

        // check default values
        assertEquals(pseudoRange, measurement.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement.getPseudoRate(), 0.0);
        assertEquals(x, measurement.getX(), 0.0);
        assertEquals(y, measurement.getY(), 0.0);
        assertEquals(z, measurement.getZ(), 0.0);
        assertEquals(vx, measurement.getVx(), 0.0);
        assertEquals(vy, measurement.getVy(), 0.0);
        assertEquals(vz, measurement.getVz(), 0.0);

        measurement = new GNSSMeasurement(pseudoRangeDistance, pseudoRateSpeed, positionAndVelocity);

        // check default values
        assertEquals(pseudoRange, measurement.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement.getPseudoRate(), 0.0);
        assertEquals(x, measurement.getX(), 0.0);
        assertEquals(y, measurement.getY(), 0.0);
        assertEquals(z, measurement.getZ(), 0.0);
        assertEquals(vx, measurement.getVx(), 0.0);
        assertEquals(vy, measurement.getVy(), 0.0);
        assertEquals(vz, measurement.getVz(), 0.0);

        // test copy constructor
        final var measurement2 = new GNSSMeasurement(measurement);

        // check default values
        assertEquals(measurement.getPseudoRange(), measurement2.getPseudoRange(), 0.0);
        assertEquals(measurement.getPseudoRate(), measurement2.getPseudoRate(), 0.0);
        assertEquals(measurement.getX(), measurement2.getX(), 0.0);
        assertEquals(measurement.getY(), measurement2.getY(), 0.0);
        assertEquals(measurement.getZ(), measurement2.getZ(), 0.0);
        assertEquals(measurement.getVx(), measurement2.getVx(), 0.0);
        assertEquals(measurement.getVy(), measurement2.getVy(), 0.0);
        assertEquals(measurement.getVz(), measurement2.getVz(), 0.0);
    }

    @Test
    void testGetSetPseudoRange() {
        final var measurement = new GNSSMeasurement();

        // check default value
        assertEquals(0.0, measurement.getPseudoRange(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        measurement.setPseudoRange(pseudoRange);

        // check
        assertEquals(pseudoRange, measurement.getPseudoRange(), 0.0);
    }

    @Test
    void testGetSetPseudoRangeDistance() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var distance1 = measurement.getPseudoRangeDistance();

        assertEquals(0.0, distance1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distance1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var distance2 = new Distance(pseudoRange, DistanceUnit.METER);
        measurement.setPseudoRangeDistance(distance2);

        // check
        final var distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        measurement.getPseudoRangeDistance(distance3);
        final var distance4 = measurement.getPseudoRangeDistance();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    void testGetSetPseudoRate() {
        final var measurement = new GNSSMeasurement();

        // check default value
        assertEquals(0.0, measurement.getPseudoRate(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        measurement.setPseudoRate(pseudoRate);

        // check
        assertEquals(pseudoRate, measurement.getPseudoRate(), 0.0);
    }

    @Test
    void testGetSetPseudoRateSpeed() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var speed1 = measurement.getPseudoRateSpeed();

        assertEquals(0.0, speed1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speed1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speed2 = new Speed(pseudoRate, SpeedUnit.METERS_PER_SECOND);
        measurement.setPseudoRateSpeed(speed2);

        // check
        final var speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        measurement.getPseudoRateSpeed(speed3);
        final var speed4 = measurement.getPseudoRateSpeed();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    void testGetSetX() {
        final var measurement = new GNSSMeasurement();

        // check default value
        assertEquals(0.0, measurement.getX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        measurement.setX(x);

        // check
        assertEquals(x, measurement.getX(), 0.0);
    }

    @Test
    void testGetSetDistanceX() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var distanceX1 = measurement.getDistanceX();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var distanceX2 = new Distance(x, DistanceUnit.METER);
        measurement.setDistanceX(distanceX2);

        // check
        final var distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        measurement.getDistanceX(distanceX3);
        final var distanceX4 = measurement.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    void testGetSetY() {
        final var measurement = new GNSSMeasurement();

        // check default value
        assertEquals(0.0, measurement.getY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        measurement.setY(y);

        // check
        assertEquals(y, measurement.getY(), 0.0);
    }

    @Test
    void testGetSetDistanceY() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var distanceY1 = measurement.getDistanceY();

        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var distanceY2 = new Distance(y, DistanceUnit.METER);
        measurement.setDistanceY(distanceY2);

        // check
        final var distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        measurement.getDistanceY(distanceY3);
        final var distanceY4 = measurement.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    void testGetSetZ() {
        final var measurement = new GNSSMeasurement();

        // check default value
        assertEquals(0.0, measurement.getZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        measurement.setZ(z);

        // check
        assertEquals(z, measurement.getZ(), 0.0);
    }

    @Test
    void testGetSetDistanceZ() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var distanceZ1 = measurement.getDistanceZ();

        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var distanceZ2 = new Distance(z, DistanceUnit.METER);
        measurement.setDistanceZ(distanceZ2);

        // check
        final var distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        measurement.getDistanceZ(distanceZ3);
        final var distanceZ4 = measurement.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    void testSetPositionCoordinates() {
        final var measurement = new GNSSMeasurement();

        // check default values
        assertEquals(0.0, measurement.getX(), 0.0);
        assertEquals(0.0, measurement.getY(), 0.0);
        assertEquals(0.0, measurement.getZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        measurement.setPositionCoordinates(x, y, z);

        // check
        assertEquals(x, measurement.getX(), 0.0);
        assertEquals(y, measurement.getY(), 0.0);
        assertEquals(z, measurement.getZ(), 0.0);
    }

    @Test
    void setPositionCoordinatesWithDistances() {
        final var measurement = new GNSSMeasurement();

        // check default values
        final var distanceX1 = measurement.getDistanceX();
        final var distanceY1 = measurement.getDistanceY();
        final var distanceZ1 = measurement.getDistanceZ();

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
        measurement.setPositionCoordinates(distanceX2, distanceY2, distanceZ2);

        // check
        final var distanceX3 = measurement.getDistanceX();
        final var distanceY3 = measurement.getDistanceY();
        final var distanceZ3 = measurement.getDistanceZ();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceZ2, distanceZ3);
    }

    @Test
    void testGetSetPosition() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var position1 = measurement.getPosition();

        // check
        assertEquals(position1, Point3D.create());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var position2 = new InhomogeneousPoint3D(x, y, z);
        measurement.setPosition(position2);

        // check
        final var position3 = measurement.getPosition();
        final var position4 = Point3D.create();
        measurement.getPosition(position4);

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetEcefPosition() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var position1 = measurement.getEcefPosition();

        // check
        assertEquals(0.0, position1.getX(), 0.0);
        assertEquals(0.0, position1.getY(), 0.0);
        assertEquals(0.0, position1.getZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var position2 = new ECEFPosition(x, y, z);
        measurement.setEcefPosition(position2);

        // check
        final var position3 = measurement.getEcefPosition();
        final var position4 = new ECEFPosition();
        measurement.getEcefPosition(position4);

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetVx() {
        final var measurement = new GNSSMeasurement();

        // check default value
        assertEquals(0.0, measurement.getVx(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        measurement.setVx(vx);

        // check
        assertEquals(vx, measurement.getVx(), 0.0);
    }

    @Test
    void testGetSetSpeedX() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var speedX1 = measurement.getSpeedX();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        measurement.setSpeedX(speedX2);

        // check
        final var speedX3 = measurement.getSpeedX();
        final var speedX4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        measurement.getSpeedX(speedX4);

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    void testGetSetVy() {
        final var measurement = new GNSSMeasurement();

        // check default value
        assertEquals(0.0, measurement.getVy(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        measurement.setVy(vy);

        // check
        assertEquals(vy, measurement.getVy(), 0.0);
    }

    @Test
    void testGetSetSpeedY() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var speedY1 = measurement.getSpeedY();

        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        measurement.setSpeedY(speedY2);

        // check
        final var speedY3 = measurement.getSpeedY();
        final var speedY4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        measurement.getSpeedY(speedY4);

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    void testGetSetVz() {
        final var measurement = new GNSSMeasurement();

        // check default value
        assertEquals(0.0, measurement.getVz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        measurement.setVz(vz);

        // check
        assertEquals(vz, measurement.getVz(), 0.0);
    }

    @Test
    void testGetSetSpeedZ() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var speedZ1 = measurement.getSpeedZ();

        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        measurement.setSpeedZ(speedZ2);

        // check
        final var speedZ3 = measurement.getSpeedZ();
        final var speedZ4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        measurement.getSpeedZ(speedZ4);

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    void testSetVelocityCoordinates() {
        final var measurement = new GNSSMeasurement();

        // check default values
        assertEquals(0.0, measurement.getVx(), 0.0);
        assertEquals(0.0, measurement.getVy(), 0.0);
        assertEquals(0.0, measurement.getVz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        measurement.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(vx, measurement.getVx(), 0.0);
        assertEquals(vy, measurement.getVy(), 0.0);
        assertEquals(vz, measurement.getVz(), 0.0);
    }

    @Test
    void testSetVelocityCoordinatesWithSpeed() {
        final var measurement = new GNSSMeasurement();

        // check default values
        final var speedX1 = measurement.getSpeedX();
        final var speedY1 = measurement.getSpeedY();
        final var speedZ1 = measurement.getSpeedZ();

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
        measurement.setVelocityCoordinates(speedX2, speedY2, speedZ2);

        // check
        final var speedX3 = measurement.getSpeedX();
        final var speedY3 = measurement.getSpeedY();
        final var speedZ3 = measurement.getSpeedZ();

        assertEquals(speedX2, speedX3);
        assertEquals(speedY2, speedY3);
        assertEquals(speedZ2, speedZ3);
    }

    @Test
    void testGetSetEcefVelocity() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var velocity1 = measurement.getEcefVelocity();

        assertEquals(0.0, velocity1.getVx(), 0.0);
        assertEquals(0.0, velocity1.getVy(), 0.0);
        assertEquals(0.0, velocity1.getVz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var velocity2 = new ECEFVelocity(vx, vy, vz);

        measurement.setEcefVelocity(velocity2);

        // check
        final var velocity3 = new ECEFVelocity();
        measurement.getEcefVelocity(velocity3);
        final var velocity4 = measurement.getEcefVelocity();

        assertEquals(velocity2, velocity3);
        assertEquals(velocity2, velocity4);
    }

    @Test
    void testGetSetPositionAndVelocity() {
        final var measurement = new GNSSMeasurement();

        // check default value
        final var positionAndVelocity1 = measurement.getPositionAndVelocity();

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

        measurement.setPositionAndVelocity(positionAndVelocity2);

        // check
        final var positionAndVelocity3 = new ECEFPositionAndVelocity();
        measurement.getPositionAndVelocity(positionAndVelocity3);
        final var positionAndVelocity4 = measurement.getPositionAndVelocity();

        assertEquals(positionAndVelocity2, positionAndVelocity3);
        assertEquals(positionAndVelocity2, positionAndVelocity4);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);

        final var measurement2 = new GNSSMeasurement();
        measurement1.copyTo(measurement2);

        // check
        assertEquals(pseudoRange, measurement2.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement2.getPseudoRate(), 0.0);
        assertEquals(x, measurement2.getX(), 0.0);
        assertEquals(y, measurement2.getY(), 0.0);
        assertEquals(z, measurement2.getZ(), 0.0);
        assertEquals(vx, measurement2.getVx(), 0.0);
        assertEquals(vy, measurement2.getVy(), 0.0);
        assertEquals(vz, measurement2.getVz(), 0.0);
        assertEquals(measurement1, measurement2);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);

        final var measurement2 = new GNSSMeasurement();
        measurement2.copyFrom(measurement1);

        // check
        assertEquals(pseudoRange, measurement2.getPseudoRange(), 0.0);
        assertEquals(pseudoRate, measurement2.getPseudoRate(), 0.0);
        assertEquals(x, measurement2.getX(), 0.0);
        assertEquals(y, measurement2.getY(), 0.0);
        assertEquals(z, measurement2.getZ(), 0.0);
        assertEquals(vx, measurement2.getVx(), 0.0);
        assertEquals(vy, measurement2.getVy(), 0.0);
        assertEquals(vz, measurement2.getVz(), 0.0);
        assertEquals(measurement1, measurement2);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);
        final var measurement2 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);
        final var measurement3 = new GNSSMeasurement();

        assertEquals(measurement1.hashCode(), measurement2.hashCode());
        assertNotEquals(measurement1.hashCode(), measurement3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);
        final var measurement2 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);
        final var measurement3 = new GNSSMeasurement();

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
        assertNotEquals(new Object(), measurement1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);
        final var measurement2 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);
        final var measurement3 = new GNSSMeasurement();

        assertTrue(measurement1.equals(measurement1, THRESHOLD));
        assertTrue(measurement1.equals(measurement2, THRESHOLD));
        assertFalse(measurement1.equals(measurement3, THRESHOLD));
        assertFalse(measurement1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);

        final var measurement2 = measurement1.clone();

        assertEquals(measurement1, measurement2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var pseudoRange = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var pseudoRate = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var x = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var y = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);
        final var z = randomizer.nextDouble(MIN_POS_VALUE, MAX_POS_VALUE);

        final var vx = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vy = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
        final var vz = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

        final var measurement1 = new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(measurement1);
        final var measurement2 = SerializationHelper.<GNSSMeasurement>deserialize(bytes);

        // check
        assertEquals(measurement1, measurement2);
        assertNotSame(measurement1, measurement2);
    }
}
