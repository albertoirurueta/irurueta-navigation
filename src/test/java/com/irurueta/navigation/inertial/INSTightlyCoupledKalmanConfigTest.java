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
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class INSTightlyCoupledKalmanConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default values
        assertEquals(config.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerBiasPSD(), 0.0, 0.0);
        assertEquals(config.getGyroBiasPSD(), 0.0, 0.0);
        assertEquals(config.getClockFrequencyPSD(), 0.0, 0.0);
        assertEquals(config.getClockPhasePSD(), 0.0, 0.0);
        assertEquals(config.getPseudoRangeSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateSD(), 0.0, 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);

        // check default values
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD, 0.0);
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);


        // test constructor with values
        final Distance pseudoRangeSDDistance = new Distance(pseudoRangeSD,
                DistanceUnit.METER);
        final Speed rangeRateSDSpeed = new Speed(rangeRateSD,
                SpeedUnit.METERS_PER_SECOND);

        config = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSDDistance, rangeRateSDSpeed);

        // check default values
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD, 0.0);
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);


        // test copy constructor
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig(config);

        // check default values
        assertEquals(config2.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config2.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config2.getAccelerometerBiasPSD(), accelerometerBiasPSD, 0.0);
        assertEquals(config2.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config2.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config2.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config2.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config2.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testGetSetGyroNoisePSD() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getGyroNoisePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setGyroNoisePSD(gyroNoisePSD);

        // check
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerNoisePSD() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getAccelerometerNoisePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setAccelerometerNoisePSD(accelerometerNoisePSD);

        // check
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(),
                0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasPSD() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getAccelerometerBiasPSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setAccelerometerBiasPSD(accelerometerBiasPSD);

        // check
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(),
                0.0);
    }

    @Test
    public void testGetSetGyroBiasPSD() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getGyroBiasPSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setGyroBiasPSD(gyroBiasPSD);

        // check
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
    }

    @Test
    public void testGetSetClockFrequencyPSD() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getClockFrequencyPSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setClockFrequencyPSD(clockFrequencyPSD);

        // check
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
    }

    @Test
    public void testGetSetClockPhasePSD() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getClockPhasePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setClockPhasePSD(clockPhasePSD);

        // check
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
    }

    @Test
    public void testGetSetPseudoRangeSD() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getPseudoRangeSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setPseudoRangeSD(pseudoRangeSD);

        // check
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
    }

    @Test
    public void testGetSetRangeRateSD() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getRangeRateSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setRangeRateSD(rangeRateSD);

        // check
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    public void testSetValues() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default values
        assertEquals(config.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerBiasPSD(), 0.0, 0.0);
        assertEquals(config.getGyroBiasPSD(), 0.0, 0.0);
        assertEquals(config.getClockFrequencyPSD(), 0.0, 0.0);
        assertEquals(config.getClockPhasePSD(), 0.0, 0.0);
        assertEquals(config.getPseudoRangeSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateSD(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setValues(gyroNoisePSD, accelerometerNoisePSD,
                accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);

        // check
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD, 0.0);
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testGetSetPseudoRangeSDDistance() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final Distance distance1 = config.getPseudoRangeSDDistance();

        assertEquals(distance1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distance1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance distance2 = new Distance(pseudoRangeSD, DistanceUnit.METER);

        config.setPseudoRangeSD(distance2);

        // check
        final Distance distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getPseudoRangeSDDistance(distance3);
        final Distance distance4 = config.getPseudoRangeSDDistance();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    public void testGetSetRangeRateSDSpeed() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default value
        final Speed speed1 = config.getRangeRateSDSpeed();

        assertEquals(speed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(speed1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speed2 = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        config.setRangeRateSD(speed2);

        // check
        final Speed speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getRangeRateSDSpeed(speed3);
        final Speed speed4 = config.getRangeRateSDSpeed();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    public void testSetValues2() {
        final INSTightlyCoupledKalmanConfig config =
                new INSTightlyCoupledKalmanConfig();

        // check default values
        assertEquals(config.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerBiasPSD(), 0.0, 0.0);
        assertEquals(config.getGyroBiasPSD(), 0.0, 0.0);
        assertEquals(config.getClockFrequencyPSD(), 0.0, 0.0);
        assertEquals(config.getClockPhasePSD(), 0.0, 0.0);
        assertEquals(config.getPseudoRangeSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateSD(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance pseudoRangeSDDistance = new Distance(pseudoRangeSD,
                DistanceUnit.METER);
        final Speed rangeRateSDSpeed = new Speed(rangeRateSD,
                SpeedUnit.METERS_PER_SECOND);

        config.setValues(gyroNoisePSD, accelerometerNoisePSD,
                accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSDDistance, rangeRateSDSpeed);

        // check
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD, 0.0);
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig();

        config1.copyTo(config2);

        // check
        assertEquals(config2.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config2.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config2.getAccelerometerBiasPSD(), accelerometerBiasPSD, 0.0);
        assertEquals(config2.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config2.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config2.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config2.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config2.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(config2.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config2.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config2.getAccelerometerBiasPSD(), accelerometerBiasPSD, 0.0);
        assertEquals(config2.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config2.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config2.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config2.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config2.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config3 =
                new INSTightlyCoupledKalmanConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config3 =
                new INSTightlyCoupledKalmanConfig();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(config1.equals((Object) config1));
        assertTrue(config1.equals(config1));
        assertTrue(config1.equals(config2));
        assertFalse(config1.equals(config3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(config1.equals((Object) null));
        assertFalse(config1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(config1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);
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
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 =
                new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                        clockPhasePSD, pseudoRangeSD, rangeRateSD);

        final Object config2 = config1.clone();

        assertEquals(config1, config2);
    }
}
