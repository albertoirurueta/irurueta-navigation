/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class StandardDeviationTimedBodyKinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor1() {
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics();

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    public void testConstructor2() {
        final BodyKinematics kinematics = createBodyKinematics();
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    public void testConstructor3() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(timestampSeconds);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    public void testConstructor4() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(time);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    public void testConstructor5() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    public void testConstructor6() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, time);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    public void testConstructor7() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                    -1.0, angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                    specificForceStandardDeviation, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor8() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, specificForceStandardDeviation,
                        angularRateStandardDeviation);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics,
                    -1.0, angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics,
                    specificForceStandardDeviation, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor9() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(timestampSeconds, specificForceStandardDeviation,
                        angularRateStandardDeviation);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(timestampSeconds,
                    -1.0, angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(timestampSeconds,
                    specificForceStandardDeviation, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor10() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(time, specificForceStandardDeviation,
                        angularRateStandardDeviation);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(time,
                    -1.0, angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(time,
                    specificForceStandardDeviation, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor11() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds,
                        specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                    kinematics, timestampSeconds,
                    -1.0, angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                    kinematics, timestampSeconds,
                    specificForceStandardDeviation, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor12() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, time,
                        specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics, time,
                    -1.0, angularRateStandardDeviation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics, time,
                    specificForceStandardDeviation, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor13() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Acceleration a = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(a, w);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                    new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND),
                    w);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                    a, new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor14() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Acceleration a = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, a,
                        w);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics,
                    new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND), w);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics,
                    a, new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor15() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Acceleration a = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(timestampSeconds, a,
                        w);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(timestampSeconds,
                    new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND), w);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(timestampSeconds,
                    a, new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor16() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);
        final Acceleration a = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(time, a, w);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(time,
                    new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND), w);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(time,
                    a, new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor17() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Acceleration a = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds,
                        a, w);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                    kinematics, timestampSeconds,
                    new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND), w);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                    kinematics, timestampSeconds,
                    a, new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testConstructor18() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);
        final Acceleration a = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics, time,
                        a, w);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        standardDeviationTimedBodyKinematics = null;
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics, time,
                    new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND), w);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics, time,
                    a, new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(standardDeviationTimedBodyKinematics);
    }

    @Test
    public void testCopyContructor() {
        final BodyKinematics kinematics = createBodyKinematics();
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);
        final Acceleration a = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics, time,
                        a, w);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics(standardDeviationTimedBodyKinematics1);


        // check default values
        assertEquals(standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics2
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), specificForceStandardDeviation, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics2
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularSpeed1.getValue().doubleValue(), angularRateStandardDeviation, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(standardDeviationTimedBodyKinematics2.getKinematics(), kinematics);
        assertEquals(standardDeviationTimedBodyKinematics2.getTimestampSeconds(), timestampSeconds, 0.0);
        final Time time1 = standardDeviationTimedBodyKinematics2.getTimestamp();
        assertEquals(time, time1);
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics2.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    public void testGetSetSpecificForceStandardDeviation() {
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics();

        // check default value
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        standardDeviationTimedBodyKinematics.setSpecificForceStandardDeviation(specificForceStandardDeviation);

        // check
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);

        // Force IllegalArgumentException
        try {
            standardDeviationTimedBodyKinematics.setSpecificForceStandardDeviation(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetSpecificForceStandardDeviationAcceleration() {
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics();

        // check default value
        final Acceleration acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();

        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final Acceleration acceleration2 = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.setSpecificForceStandardDeviation(acceleration2);

        // check
        final Acceleration acceleration3 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        final Acceleration acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);

        // Force IllegalArgumentException
        try {
            standardDeviationTimedBodyKinematics.setSpecificForceStandardDeviation(
                    new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND));
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAngularRateStandardDeviation() {
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics();

        // check default value
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        standardDeviationTimedBodyKinematics.setAngularRateStandardDeviation(angularRateStandardDeviation);

        // check
        assertEquals(standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(),
                angularRateStandardDeviation, 0.0);

        // Force IllegalArgumentException
        try {
            standardDeviationTimedBodyKinematics.setAngularRateStandardDeviation(-1.0);
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAngularRateStandardDeviationAsAngularSpeed() {
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics();

        // check default value
        final AngularSpeed angularSpeed1 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();

        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final AngularSpeed angularSpeed2 = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        standardDeviationTimedBodyKinematics.setAngularRateStandardDeviation(angularSpeed2);

        // check
        final AngularSpeed angularSpeed3 = standardDeviationTimedBodyKinematics
                .getAngularRateStandardDeviationAsAngularSpeed();
        final AngularSpeed angularSpeed4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    public void testGetSetKinematics() {
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics();

        // check default value
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());

        // set new value
        final BodyKinematics kinematics = new BodyKinematics();
        standardDeviationTimedBodyKinematics.setKinematics(kinematics);

        // check
        assertSame(standardDeviationTimedBodyKinematics.getKinematics(), kinematics);
    }

    @Test
    public void testGetSetTimestampSeconds() {
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics();

        // check default value
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        standardDeviationTimedBodyKinematics.setTimestampSeconds(timestampSeconds);

        // check
        assertEquals(standardDeviationTimedBodyKinematics.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testGetSetTimestamp() {
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics =
                new StandardDeviationTimedBodyKinematics();

        // check default value
        final Time time1 = standardDeviationTimedBodyKinematics.getTimestamp();

        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final Time time2 = new Time(timestampSeconds, TimeUnit.SECOND);
        standardDeviationTimedBodyKinematics.setTimestamp(time2);

        final Time time3 = standardDeviationTimedBodyKinematics.getTimestamp();
        final Time time4 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time4);

        // check
        assertEquals(time2, time3);
        assertEquals(time2, time4);
    }

    @Test
    public void testCopyFrmmWhenBodyKinematicsIsAvailableAtSourceAndDestinationIsEmpty() {
        final BodyKinematics kinematics = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds,
                        specificForceStandardDeviation, angularRateStandardDeviation);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics();

        standardDeviationTimedBodyKinematics2.copyFrom(standardDeviationTimedBodyKinematics1);

        // check
        assertEquals(kinematics, standardDeviationTimedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics2.getTimestampSeconds(), 0.0);
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics1.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    public void testCopyFromWhenDestinationHasKinematics() {
        final BodyKinematics kinematics = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics();
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds,
                        specificForceStandardDeviation, angularRateStandardDeviation);

        standardDeviationTimedBodyKinematics2.copyFrom(standardDeviationTimedBodyKinematics1);

        // check
        assertNull(standardDeviationTimedBodyKinematics2.getKinematics());
        assertEquals(standardDeviationTimedBodyKinematics2.getTimestampSeconds(), 0.0, 0.0);
        assertEquals(standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(), 0.0, 0.0);
        assertEquals(standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviation(), 0.0, 0.0);
    }

    @Test
    public void testCopyFromWhenSourceAndDestinationHasKinematics() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final double specificForceStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double specificForceStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final double angularRateStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final double angularRateStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics(kinematics2, timestampSeconds2,
                        specificForceStandardDeviation2, angularRateStandardDeviation2);

        standardDeviationTimedBodyKinematics2.copyFrom(standardDeviationTimedBodyKinematics1);

        // check
        assertEquals(standardDeviationTimedBodyKinematics2.getKinematics(), kinematics1);
        assertEquals(standardDeviationTimedBodyKinematics2.getTimestampSeconds(), timestampSeconds1, 0.0);
        assertEquals(standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation1, 0.0);
        assertEquals(standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviation(),
                angularRateStandardDeviation1, 0.0);
    }

    @Test
    public void testCopyTo() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final double specificForceStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double specificForceStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final double angularRateStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final double angularRateStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics(kinematics2, timestampSeconds2,
                        specificForceStandardDeviation2, angularRateStandardDeviation2);

        standardDeviationTimedBodyKinematics1.copyTo(standardDeviationTimedBodyKinematics2);

        // check
        assertEquals(standardDeviationTimedBodyKinematics2.getKinematics(), kinematics1);
        assertEquals(standardDeviationTimedBodyKinematics2.getTimestampSeconds(), timestampSeconds1, 0.0);
        assertEquals(standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation1, 0.0);
        assertEquals(standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviation(),
                angularRateStandardDeviation1, 0.0);
    }

    @Test
    public void testHashCode() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final double specificForceStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double specificForceStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final double angularRateStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final double angularRateStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics3 =
                new StandardDeviationTimedBodyKinematics(kinematics2, timestampSeconds2,
                        specificForceStandardDeviation2, angularRateStandardDeviation2);

        assertEquals(standardDeviationTimedBodyKinematics1.hashCode(),
                standardDeviationTimedBodyKinematics2.hashCode());
        assertNotEquals(standardDeviationTimedBodyKinematics1.hashCode(),
                standardDeviationTimedBodyKinematics3.hashCode());
    }

    @Test
    public void testEquals() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final double specificForceStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double specificForceStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final double angularRateStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final double angularRateStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics3 =
                new StandardDeviationTimedBodyKinematics(kinematics2, timestampSeconds2,
                        specificForceStandardDeviation2, angularRateStandardDeviation2);

        // noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(standardDeviationTimedBodyKinematics1.equals((Object) standardDeviationTimedBodyKinematics1));
        assertTrue(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics1));
        assertTrue(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics2));
        assertFalse(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics3));
        // noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(standardDeviationTimedBodyKinematics1.equals((Object) null));
        assertFalse(standardDeviationTimedBodyKinematics1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(standardDeviationTimedBodyKinematics1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final double specificForceStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double specificForceStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final double angularRateStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final double angularRateStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics2 =
                new StandardDeviationTimedBodyKinematics(kinematics1, timestampSeconds1,
                        specificForceStandardDeviation1, angularRateStandardDeviation1);
        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics3 =
                new StandardDeviationTimedBodyKinematics(kinematics2, timestampSeconds2,
                        specificForceStandardDeviation2, angularRateStandardDeviation2);

        assertTrue(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics1, THRESHOLD));
        assertTrue(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics2, THRESHOLD));
        assertFalse(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics3, THRESHOLD));
        assertFalse(standardDeviationTimedBodyKinematics1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final BodyKinematics kinematics = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double specificForceStandardDeviation =
                randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation =
                randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationTimedBodyKinematics standardDeviationTimedBodyKinematics1 =
                new StandardDeviationTimedBodyKinematics(kinematics, timestampSeconds,
                        specificForceStandardDeviation, angularRateStandardDeviation);

        final Object standardDeviationTimedBodyKinematics2 = standardDeviationTimedBodyKinematics1.clone();

        // check
        assertEquals(standardDeviationTimedBodyKinematics1, standardDeviationTimedBodyKinematics2);
    }

    private BodyKinematics createBodyKinematics() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        return new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }
}
