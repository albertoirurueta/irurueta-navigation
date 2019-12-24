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
package com.irurueta.navigation.inertial;

import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;
import static org.junit.Assert.assertFalse;

public class INSTightlyCoupledKalmanConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {

        // test empty constructor
        INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default values
        assertEquals(config.getInitialAttitudeUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialVelocityUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialPositionUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                0.0, 0.0);
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                0.0, 0.0);
        assertEquals(config.getInitialClockOffsetUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialClockDriftUncertainty(), 0.0, 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config = new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty);

        // check default values
        assertEquals(config.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
        assertEquals(config.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
        assertEquals(config.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
        assertEquals(config.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
        assertEquals(config.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);


        // test constructor with measurement values
        final Angle initialAttitudeUncertaintyAngle =
                new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        final Speed initialVelocityUncertaintySpeed =
                new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        final Distance initialPositionUncertaintyDistance =
                new Distance(initialPositionUncertainty, DistanceUnit.METER);
        final Acceleration initialAccelerationBiasUncertaintyAcceleration =
                new Acceleration(initialAccelerationBiasUncertainty,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed initialGyroscopeBiasUncertaintyAngularSpeed =
                new AngularSpeed(initialGyroscopeBiasUncertainty,
                        AngularSpeedUnit.RADIANS_PER_SECOND);
        final Distance initialClockOffsetUncertaintyDistance =
                new Distance(initialClockOffsetUncertainty, DistanceUnit.METER);
        final Speed initialClockDriftUncertaintySpeed =
                new Speed(initialClockDriftUncertainty, SpeedUnit.METERS_PER_SECOND);
        config = new INSTightlyCoupledKalmanConfig(initialAttitudeUncertaintyAngle,
                initialVelocityUncertaintySpeed, initialPositionUncertaintyDistance,
                initialAccelerationBiasUncertaintyAcceleration,
                initialGyroscopeBiasUncertaintyAngularSpeed,
                initialClockOffsetUncertaintyDistance,
                initialClockDriftUncertaintySpeed);

        // check default values
        assertEquals(config.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
        assertEquals(config.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
        assertEquals(config.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
        assertEquals(config.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
        assertEquals(config.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);


        // test copy constructor
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig(config);

        // check default values
        assertEquals(config2.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
        assertEquals(config2.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
        assertEquals(config2.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
        assertEquals(config2.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
        assertEquals(config2.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
        assertEquals(config2.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
        assertEquals(config2.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);
    }

    @Test
    public void testGetSetInitialAttitudeUncertainty() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialAttitudeUncertainty(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialAttitudeUncertainty(initialAttitudeUncertainty);

        // check
        assertEquals(config.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
    }

    @Test
    public void testGetSetInitialAttitudeUncertaintyAngle() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final Angle initialAttitudeUncertainty1 =
                config.getInitialAttitudeUncertaintyAngle();

        assertEquals(initialAttitudeUncertainty1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialAttitudeUncertainty1.getUnit(),
                AngleUnit.RADIANS);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Angle initialAttitudeUncertainty2 =
                new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        config.setInitialAttitudeUncertainty(initialAttitudeUncertainty2);

        // check
        final Angle initialAttitudeUncertainty3 =
                config.getInitialAttitudeUncertaintyAngle();
        final Angle initialAttitudeUncertainty4 = new Angle(0.0,
                AngleUnit.DEGREES);
        config.getInitialAttitudeUncertaintyAngle(initialAttitudeUncertainty4);

        assertEquals(initialAttitudeUncertainty2, initialAttitudeUncertainty3);
        assertEquals(initialAttitudeUncertainty2, initialAttitudeUncertainty4);
    }

    @Test
    public void testGetSetInitialVelocityUncertainty() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialVelocityUncertainty(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty);

        // check
        assertEquals(config.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
    }

    @Test
    public void testGetSetInitialVelocityUncertaintySpeed() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final Speed initialVelocityUncertaintySpeed1 =
                config.getInitialVelocityUncertaintySpeed();

        assertEquals(initialVelocityUncertaintySpeed1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialVelocityUncertaintySpeed1.getUnit(),
                SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Speed initialVelocityUncertainty2 =
                new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty2);

        // check
        final Speed initialVelocityUncertainty3 =
                config.getInitialVelocityUncertaintySpeed();
        final Speed initialVelocityUncertainty4 = new Speed(0.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        config.getInitialVelocityUncertaintySpeed(initialVelocityUncertainty4);

        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty3);
        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty4);
    }

    @Test
    public void testGetSetInitialPositionUncertainty() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialPositionUncertainty(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialPositionUncertainty(initialPositionUncertainty);

        // check
        assertEquals(config.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
    }

    @Test
    public void testGetSetInitialPositionUncertaintyDistance() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final Distance initialPositionUncertainty1 =
                config.getInitialPositionUncertaintyDistance();

        assertEquals(initialPositionUncertainty1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialPositionUncertainty1.getUnit(),
                DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Distance initialPositionUncertainty2 =
                new Distance(initialPositionUncertainty, DistanceUnit.METER);
        config.setInitialPositionUncertainty(initialPositionUncertainty2);

        // check
        final Distance initialPositionUncertainty3 =
                config.getInitialPositionUncertaintyDistance();
        final Distance initialPositionUncertainty4 = new Distance(0.0,
                DistanceUnit.KILOMETER);
        config.getInitialPositionUncertaintyDistance(initialPositionUncertainty4);

        assertEquals(initialPositionUncertainty2, initialPositionUncertainty3);
        assertEquals(initialPositionUncertainty2, initialPositionUncertainty4);
    }

    @Test
    public void testGetSetInitialAccelerationBiasUncertainty() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialAccelerationBiasUncertainty(
                initialAccelerationBiasUncertainty);

        // check
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
    }

    @Test
    public void testGetSetInitialAccelerationBiasUncertaintyAcceleration() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final Acceleration initialAccelerationBiasUncertainty1 =
                config.getInitialAccelerationBiasUncertaintyAcceleration();

        assertEquals(initialAccelerationBiasUncertainty1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialAccelerationBiasUncertainty1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Acceleration initialAccelerationBiasUncertainty2 =
                new Acceleration(initialAccelerationBiasUncertainty,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        config.setInitialAccelerationBiasUncertainty(
                initialAccelerationBiasUncertainty2);

        // check
        final Acceleration initialAccelerationBiasUncertainty3 =
                config.getInitialAccelerationBiasUncertaintyAcceleration();
        final Acceleration initialAccelerationBiasUncertainty4 =
                new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        config.getInitialAccelerationBiasUncertaintyAcceleration(
                initialAccelerationBiasUncertainty4);

        assertEquals(initialAccelerationBiasUncertainty2,
                initialAccelerationBiasUncertainty3);
        assertEquals(initialAccelerationBiasUncertainty2,
                initialAccelerationBiasUncertainty4);
    }

    @Test
    public void testGetSetInitialGyroscopeBiasUncertainty() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialGyroscopeBiasUncertainty(
                initialGyroscopeBiasUncertainty);

        // check
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
    }

    @Test
    public void testGetSetInitialGyroscopeBiasUncertaintyAngularSpeed() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final AngularSpeed initialGyroscopeBiasUncertainty1 =
                config.getInitialGyroscopeBiasUncertaintyAngularSpeed();

        assertEquals(initialGyroscopeBiasUncertainty1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialGyroscopeBiasUncertainty1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final AngularSpeed initialGyroscopeBiasUncertainty2 =
                new AngularSpeed(initialGyroscopeBiasUncertainty,
                        AngularSpeedUnit.RADIANS_PER_SECOND);
        config.setInitialGyroscopeBiasUncertainty(
                initialGyroscopeBiasUncertainty2);

        // check
        final AngularSpeed initialGyroscopeBiasUncertainty3 =
                config.getInitialGyroscopeBiasUncertaintyAngularSpeed();
        final AngularSpeed initialGyroscopeBiasUncertainty4 =
                new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        config.getInitialGyroscopeBiasUncertaintyAngularSpeed(
                initialGyroscopeBiasUncertainty4);

        assertEquals(initialGyroscopeBiasUncertainty2,
                initialGyroscopeBiasUncertainty3);
        assertEquals(initialGyroscopeBiasUncertainty2,
                initialGyroscopeBiasUncertainty4);
    }

    @Test
    public void testGetSetInitialClockOffsetUncertainty() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialClockOffsetUncertainty(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialClockOffsetUncertainty(initialClockOffsetUncertainty);

        // check
        assertEquals(config.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
    }

    @Test
    public void testGetSetInitialClockOffsetUncertaintyDistance() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final Distance initialClockOffsetUncertainty1 =
                config.getInitialClockOffsetUncertaintyDistance();

        assertEquals(initialClockOffsetUncertainty1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialClockOffsetUncertainty1.getUnit(),
                DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Distance initialClockOffsetUncertainty2 =
                new Distance(initialClockOffsetUncertainty, DistanceUnit.METER);
        config.setInitialClockOffsetUncertainty(initialClockOffsetUncertainty2);

        // check
        final Distance initialClockOffsetUncertainty3 =
                config.getInitialClockOffsetUncertaintyDistance();
        final Distance initialClockOffsetUncertainty4 =
                new Distance(0.0, DistanceUnit.METER);
        config.getInitialClockOffsetUncertaintyDistance(
                initialClockOffsetUncertainty4);

        assertEquals(initialClockOffsetUncertainty2,
                initialClockOffsetUncertainty3);
        assertEquals(initialClockOffsetUncertainty2,
                initialClockOffsetUncertainty4);
    }

    @Test
    public void testGetSetInitialClockDriftUncertainty() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialClockDriftUncertainty(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialClockDriftUncertainty(
                initialClockDriftUncertainty);

        // check
        assertEquals(config.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);
    }

    @Test
    public void testGetSetInitialClockDriftUncertaintySpeed() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final Speed initialClockDriftUncertainty1 =
                config.getInitialClockDriftUncertaintySpeed();

        assertEquals(initialClockDriftUncertainty1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialClockDriftUncertainty1.getUnit(),
                SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Speed initialClockDriftUncertainty2 =
                new Speed(initialClockDriftUncertainty,
                        SpeedUnit.METERS_PER_SECOND);
        config.setInitialClockDriftUncertainty(
                initialClockDriftUncertainty2);

        // check
        final Speed initialClockDriftUncertainty3 =
                config.getInitialClockDriftUncertaintySpeed();
        final Speed initialClockDriftUncertainty4 =
                new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getInitialClockDriftUncertaintySpeed(
                initialClockDriftUncertainty4);

        assertEquals(initialClockDriftUncertainty2, initialClockDriftUncertainty3);
        assertEquals(initialClockDriftUncertainty2, initialClockDriftUncertainty4);
    }

    @Test
    public void testSetValues() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialAttitudeUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialVelocityUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialPositionUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                0.0, 0.0);
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                0.0, 0.0);
        assertEquals(config.getInitialClockOffsetUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialClockDriftUncertainty(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setValues(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty);

        // check
        assertEquals(config.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
        assertEquals(config.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
        assertEquals(config.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
        assertEquals(config.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
        assertEquals(config.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);
    }

    @Test
    public void testSetValues2() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getInitialAttitudeUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialVelocityUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialPositionUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                0.0, 0.0);
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                0.0, 0.0);
        assertEquals(config.getInitialClockOffsetUncertainty(), 0.0, 0.0);
        assertEquals(config.getInitialClockDriftUncertainty(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        final Angle initialAttitudeUncertaintyAngle =
                new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        final Speed initialVelocityUncertaintySpeed =
                new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        final Distance initialPositionUncertaintyDistance =
                new Distance(initialPositionUncertainty, DistanceUnit.METER);
        final Acceleration initialAccelerationBiasUncertaintyAcceleration =
                new Acceleration(initialAccelerationBiasUncertainty,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed initialGyroscopeBiasUncertaintyAngularSpeed =
                new AngularSpeed(initialGyroscopeBiasUncertainty,
                        AngularSpeedUnit.RADIANS_PER_SECOND);
        final Distance initialClockOffsetUncertaintyDistance =
                new Distance(initialClockOffsetUncertainty, DistanceUnit.METER);
        final Speed initialClockDriftUncertaintySpeed =
                new Speed(initialClockDriftUncertainty, SpeedUnit.METERS_PER_SECOND);

        config.setValues(initialAttitudeUncertaintyAngle,
                initialVelocityUncertaintySpeed, initialPositionUncertaintyDistance,
                initialAccelerationBiasUncertaintyAcceleration,
                initialGyroscopeBiasUncertaintyAngularSpeed,
                initialClockOffsetUncertaintyDistance,
                initialClockDriftUncertaintySpeed);

        // check
        assertEquals(config.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
        assertEquals(config.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
        assertEquals(config.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
        assertEquals(config.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
        assertEquals(config.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);

        // check
        assertEquals(config.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
        assertEquals(config.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
        assertEquals(config.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
        assertEquals(config.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
        assertEquals(config.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
        assertEquals(config.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
        assertEquals(config.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty);

        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig();

        config1.copyTo(config2);

        // check
        assertEquals(config2.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
        assertEquals(config2.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
        assertEquals(config2.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
        assertEquals(config2.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
        assertEquals(config2.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
        assertEquals(config2.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
        assertEquals(config2.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                        initialVelocityUncertainty, initialPositionUncertainty,
                        initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                        initialClockOffsetUncertainty, initialClockDriftUncertainty);

        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(config2.getInitialAttitudeUncertainty(),
                initialAttitudeUncertainty, 0.0);
        assertEquals(config2.getInitialVelocityUncertainty(),
                initialVelocityUncertainty, 0.0);
        assertEquals(config2.getInitialPositionUncertainty(),
                initialPositionUncertainty, 0.0);
        assertEquals(config2.getInitialAccelerationBiasUncertainty(),
                initialAccelerationBiasUncertainty, 0.0);
        assertEquals(config2.getInitialGyroscopeBiasUncertainty(),
                initialGyroscopeBiasUncertainty, 0.0);
        assertEquals(config2.getInitialClockOffsetUncertainty(),
                initialClockOffsetUncertainty, 0.0);
        assertEquals(config2.getInitialClockDriftUncertainty(),
                initialClockDriftUncertainty, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                        initialVelocityUncertainty, initialPositionUncertainty,
                        initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                        initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                        initialVelocityUncertainty, initialPositionUncertainty,
                        initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                        initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final INSTightlyCoupledKalmanConfig config3 =
                new INSTightlyCoupledKalmanConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                        initialVelocityUncertainty, initialPositionUncertainty,
                        initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                        initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                        initialVelocityUncertainty, initialPositionUncertainty,
                        initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                        initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final INSTightlyCoupledKalmanConfig config3 =
                new INSTightlyCoupledKalmanConfig();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(config1.equals((Object)config1));
        assertTrue(config1.equals(config1));
        assertTrue(config1.equals(config2));
        assertFalse(config1.equals(config3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(config1.equals((Object)null));
        assertFalse(config1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(config1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                        initialVelocityUncertainty, initialPositionUncertainty,
                        initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                        initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                        initialVelocityUncertainty, initialPositionUncertainty,
                        initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                        initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final INSTightlyCoupledKalmanConfig config3 =
                new INSTightlyCoupledKalmanConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(initialAttitudeUncertainty,
                        initialVelocityUncertainty, initialPositionUncertainty,
                        initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                        initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final Object config2 = config1.clone();

        assertEquals(config1, config2);
    }
}
