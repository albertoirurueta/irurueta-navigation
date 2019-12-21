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

public class GNSSLeastSquaresPositionAndVelocityEstimatorResultTest {

    private static final double MIN_VALUE = -1.0;
    private static final double MAX_VALUE = 1.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {

        // test empty constructor
        GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default values
        assertEquals(result.getX(), 0.0, 0.0);
        assertEquals(result.getY(), 0.0, 0.0);
        assertEquals(result.getZ(), 0.0, 0.0);
        assertEquals(result.getVx(), 0.0, 0.0);
        assertEquals(result.getVy(), 0.0, 0.0);
        assertEquals(result.getVz(), 0.0, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(), 0.0, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(), 0.0, 0.0);


        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        result = new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);

        // check default values
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);


        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);
        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        final Distance estimatedReceiverClockOffsetDistance =
                new Distance(estimatedReceiverClockOffset, DistanceUnit.METER);
        final Speed estimatedReceiverClockDriftSpeed =
                new Speed(estimatedReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);
        result = new GNSSLeastSquaresPositionAndVelocityEstimatorResult(
                distanceX, distanceY, distanceZ, speedX, speedY, speedZ,
                estimatedReceiverClockOffsetDistance,
                estimatedReceiverClockDriftSpeed);

        // check default values
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);


        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        result = new GNSSLeastSquaresPositionAndVelocityEstimatorResult(position,
                vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);

        // check default values
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);


        result = new GNSSLeastSquaresPositionAndVelocityEstimatorResult(position,
                speedX, speedY, speedZ, estimatedReceiverClockOffsetDistance,
                estimatedReceiverClockDriftSpeed);

        // check default value
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);


        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);
        final ECEFVelocity ecefVelocity = new ECEFVelocity(vx, vy, vz);
        result = new GNSSLeastSquaresPositionAndVelocityEstimatorResult(
                ecefPosition, ecefVelocity, estimatedReceiverClockOffset,
                estimatedReceiverClockDrift);

        // check default value
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);


        result = new GNSSLeastSquaresPositionAndVelocityEstimatorResult(
                ecefPosition, ecefVelocity, estimatedReceiverClockOffsetDistance,
                estimatedReceiverClockDriftSpeed);

        // check default value
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);


        final ECEFPositionAndVelocity positionAndVelocity =
                new ECEFPositionAndVelocity(ecefPosition, ecefVelocity);
        result = new GNSSLeastSquaresPositionAndVelocityEstimatorResult(
                positionAndVelocity, estimatedReceiverClockOffset,
                estimatedReceiverClockDrift);

        // check default value
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);


        result = new GNSSLeastSquaresPositionAndVelocityEstimatorResult(
                positionAndVelocity, estimatedReceiverClockOffsetDistance,
                estimatedReceiverClockDriftSpeed);

        // check default value
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);
    }

    @Test
    public void testGetSetX() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        assertEquals(result.getX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        result.setX(x);

        // check
        assertEquals(result.getX(), x, 0.0);
    }

    @Test
    public void testGetSetY() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        assertEquals(result.getY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        result.setY(y);

        // check
        assertEquals(result.getY(), y, 0.0);
    }

    @Test
    public void testGetSetZ() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        assertEquals(result.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        result.setZ(z);

        // check
        assertEquals(result.getZ(), z, 0.0);
    }

    @Test
    public void testSetPositionCoordinates() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default values
        assertEquals(result.getX(), 0.0, 0.0);
        assertEquals(result.getY(), 0.0, 0.0);
        assertEquals(result.getZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        result.setPositionCoordinates(x, y, z);

        // check
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetVx() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default values
        assertEquals(result.getVx(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        result.setVx(vx);

        // check
        assertEquals(result.getVx(), vx, 0.0);
    }

    @Test
    public void testGetSetVy() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default values
        assertEquals(result.getVy(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        result.setVy(vy);

        // check
        assertEquals(result.getVy(), vy, 0.0);
    }

    @Test
    public void testGetSetVz() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default values
        assertEquals(result.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        result.setVz(vz);

        // check
        assertEquals(result.getVz(), vz, 0.0);
    }

    @Test
    public void testSetVelocityCoordinates() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default values
        assertEquals(result.getVx(), 0.0, 0.0);
        assertEquals(result.getVy(), 0.0, 0.0);
        assertEquals(result.getVz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        result.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetEstimatedReceiverClockOffset() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        assertEquals(result.getEstimatedReceiverClockOffset(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        result.setEstimatedReceiverClockOffset(estimatedReceiverClockOffset);

        // check
        assertEquals(result.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
    }

    @Test
    public void testGetSetEstimatedReceiverClockDrift() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        assertEquals(result.getEstimatedReceiverClockDrift(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        result.setEstimatedReceiverClockDrift(estimatedReceiverClockDrift);

        // check
        assertEquals(result.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);
    }

    @Test
    public void testGetSetDistanceX() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Distance distance1 = result.getDistanceX();
        assertEquals(distance1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distance1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance distance2 = new Distance(x, DistanceUnit.METER);

        result.setDistanceX(distance2);

        // check
        final Distance distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        result.getDistanceX(distance3);
        final Distance distance4 = result.getDistanceX();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    public void testGetSetDistanceY() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Distance distance1 = result.getDistanceY();
        assertEquals(distance1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distance1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance distance2 = new Distance(y, DistanceUnit.METER);

        result.setDistanceY(distance2);

        // check
        final Distance distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        result.getDistanceY(distance3);
        final Distance distance4 = result.getDistanceY();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    public void testGetSetDistanceZ() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Distance distance1 = result.getDistanceZ();
        assertEquals(distance1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distance1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance distance2 = new Distance(z, DistanceUnit.METER);

        result.setDistanceZ(distance2);

        // check
        final Distance distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        result.getDistanceZ(distance3);
        final Distance distance4 = result.getDistanceZ();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    public void testSetPositionCoordinatesDistance() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        assertEquals(result.getX(), 0.0, 0.0);
        assertEquals(result.getY(), 0.0, 0.0);
        assertEquals(result.getZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        result.setPositionCoordinates(distanceX, distanceY, distanceZ);

        // check
        assertEquals(result.getX(), x, 0.0);
        assertEquals(result.getY(), y, 0.0);
        assertEquals(result.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetSpeedX() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Speed speed1 = result.getSpeedX();

        assertEquals(speed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speed1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speed2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        result.setSpeedX(speed2);

        // check
        final Speed speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        result.getSpeedX(speed3);
        final Speed speed4 = result.getSpeedX();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    public void testGetSetSpeedY() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Speed speed1 = result.getSpeedY();

        assertEquals(speed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speed1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speed2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        result.setSpeedY(speed2);

        // check
        final Speed speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        result.getSpeedY(speed3);
        final Speed speed4 = result.getSpeedY();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    public void testGetSetSpeedZ() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Speed speed1 = result.getSpeedZ();

        assertEquals(speed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speed1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speed2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        result.setSpeedZ(speed2);

        // check
        final Speed speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        result.getSpeedZ(speed3);
        final Speed speed4 = result.getSpeedZ();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    public void testSetVelocityCoordinatesSpeed() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default values
        assertEquals(result.getVx(), 0.0, 0.0);
        assertEquals(result.getVy(), 0.0, 0.0);
        assertEquals(result.getVz(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        result.setVelocityCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(result.getVx(), vx, 0.0);
        assertEquals(result.getVy(), vy, 0.0);
        assertEquals(result.getVz(), vz, 0.0);
    }

    @Test
    public void testGetSetEstimatedReceiverClockOffsetDistance() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Distance distance1 = result.getEstimatedReceiverClockOffsetDistance();

        assertEquals(distance1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distance1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance distance2 = new Distance(estimatedReceiverClockOffset,
                DistanceUnit.METER);
        result.setEstimatedReceiverClockOffset(distance2);

        // check
        final Distance distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        result.getEstimatedReceiverClockOffsetDistance(distance3);
        final Distance distance4 = result.getEstimatedReceiverClockOffsetDistance();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    public void testGetSetEstimatedReceiverClockDriftSpeed() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Speed speed1 = result.getEstimatedReceiverClockDriftSpeed();

        assertEquals(speed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speed1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Speed speed2 = new Speed(estimatedReceiverClockDrift,
                SpeedUnit.METERS_PER_SECOND);
        result.setEstimatedReceiverClockDrift(speed2);

        // check
        final Speed speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        result.getEstimatedReceiverClockDriftSpeed(speed3);
        final Speed speed4 = result.getEstimatedReceiverClockDriftSpeed();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    public void testGetSetPosition() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final Point3D position1 = result.getPosition();
        assertEquals(position1, Point3D.create());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Point3D position2 = new InhomogeneousPoint3D(x, y, z);
        result.setPosition(position2);

        // check
        final Point3D position3 = new InhomogeneousPoint3D();
        result.getPosition(position3);
        final Point3D position4 = result.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetEcefPosition() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final ECEFPosition position1 = result.getEcefPosition();

        assertEquals(position1.getX(), 0.0, 0.0);
        assertEquals(position1.getY(), 0.0, 0.0);
        assertEquals(position1.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFPosition position2 = new ECEFPosition(x, y, z);
        result.setEcefPosition(position2);

        // check
        final ECEFPosition position3 = new ECEFPosition();
        result.getEcefPosition(position3);
        final ECEFPosition position4 = result.getEcefPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetEcefVelocity() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final ECEFVelocity velocity1 = result.getEcefVelocity();

        assertEquals(velocity1.getVx(), 0.0, 0.0);
        assertEquals(velocity1.getVy(), 0.0, 0.0);
        assertEquals(velocity1.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFVelocity velocity2 = new ECEFVelocity(vx, vy, vz);
        result.setEcefVelocity(velocity2);

        // check
        final ECEFVelocity velocity3 = new ECEFVelocity();
        result.getEcefVelocity(velocity3);
        final ECEFVelocity velocity4 = result.getEcefVelocity();

        assertEquals(velocity2, velocity3);
        assertEquals(velocity2, velocity4);
    }

    @Test
    public void testGetSetPositionAndVelocity() {
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        // check default value
        final ECEFPositionAndVelocity positionAndVelocity1 =
                result.getPositionAndVelocity();

        assertEquals(positionAndVelocity1.getX(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getY(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getZ(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getVx(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getVy(), 0.0, 0.0);
        assertEquals(positionAndVelocity1.getVz(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final ECEFPositionAndVelocity positionAndVelocity2 =
                new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        result.setPositionAndVelocity(positionAndVelocity2);

        // check
        final ECEFPositionAndVelocity positionAndVelocity3 =
                new ECEFPositionAndVelocity();
        result.getPositionAndVelocity(positionAndVelocity3);
        final ECEFPositionAndVelocity positionAndVelocity4 =
                result.getPositionAndVelocity();

        assertEquals(positionAndVelocity2, positionAndVelocity3);
        assertEquals(positionAndVelocity2, positionAndVelocity4);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result1 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result2 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        result1.copyTo(result2);

        // check
        assertEquals(result2.getX(), x, 0.0);
        assertEquals(result2.getY(), y, 0.0);
        assertEquals(result2.getZ(), z, 0.0);
        assertEquals(result2.getVx(), vx, 0.0);
        assertEquals(result2.getVy(), vy, 0.0);
        assertEquals(result2.getVz(), vz, 0.0);
        assertEquals(result2.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result2.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result1 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                        vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result2 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        result2.copyFrom(result1);

        // check
        assertEquals(result2.getX(), x, 0.0);
        assertEquals(result2.getY(), y, 0.0);
        assertEquals(result2.getZ(), z, 0.0);
        assertEquals(result2.getVx(), vx, 0.0);
        assertEquals(result2.getVy(), vy, 0.0);
        assertEquals(result2.getVz(), vz, 0.0);
        assertEquals(result2.getEstimatedReceiverClockOffset(),
                estimatedReceiverClockOffset, 0.0);
        assertEquals(result2.getEstimatedReceiverClockDrift(),
                estimatedReceiverClockDrift, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result1 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                        vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result2 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                        vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result3 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        assertEquals(result1.hashCode(), result2.hashCode());
        assertNotEquals(result1.hashCode(), result3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result1 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                        vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result2 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                        vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result3 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(result1.equals((Object)result1));
        assertTrue(result1.equals(result1));
        assertTrue(result1.equals(result2));
        assertFalse(result1.equals(result3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(result1.equals((Object)null));
        assertFalse(result1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(result1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result1 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                        vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result2 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                        vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);
        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result3 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult();

        assertTrue(result1.equals(result1, THRESHOLD));
        assertTrue(result1.equals(result2, THRESHOLD));
        assertFalse(result1.equals(result3, THRESHOLD));
        assertFalse(result1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockOffset =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double estimatedReceiverClockDrift =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSLeastSquaresPositionAndVelocityEstimatorResult result1 =
                new GNSSLeastSquaresPositionAndVelocityEstimatorResult(x, y, z,
                        vx, vy, vz, estimatedReceiverClockOffset, estimatedReceiverClockDrift);

        final Object result2 = result1.clone();

        assertEquals(result1, result2);
    }
}
