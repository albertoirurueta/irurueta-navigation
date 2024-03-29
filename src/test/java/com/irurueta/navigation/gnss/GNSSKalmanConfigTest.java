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

import com.irurueta.navigation.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class GNSSKalmanConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default values
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialClockDriftUncertainty(), 0.0);
        assertEquals(0.0, config.getAccelerationPSD(), 0.0);
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);
        assertEquals(0.0, config.getRangeRateSD(), 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);

        // check default values
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
        assertEquals(accelerationPSD, config.getAccelerationPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);


        //test other constructor with values
        final Distance distancePseudoRangeSD = new Distance(pseudoRangeSD, DistanceUnit.METER);
        final Speed speedRangeRateSD = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        final Distance distanceInitialPositionUncertainty = new Distance(
                initialPositionUncertainty, DistanceUnit.METER);
        final Speed speedInitialVelocityUncertainty = new Speed(
                initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        final Distance distanceInitialClockOffsetUncertainty = new Distance(
                initialClockOffsetUncertainty, DistanceUnit.METER);
        final Speed speedInitialClockDriftUncertainty = new Speed(
                initialClockDriftUncertainty, SpeedUnit.METERS_PER_SECOND);

        config = new GNSSKalmanConfig(distanceInitialPositionUncertainty, speedInitialVelocityUncertainty,
                distanceInitialClockOffsetUncertainty, speedInitialClockDriftUncertainty, accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, distancePseudoRangeSD, speedRangeRateSD);

        // check default values
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
        assertEquals(accelerationPSD, config.getAccelerationPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);

        // test copy constructor
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig(config);

        // check default values
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config2.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config2.getInitialClockDriftUncertainty(), 0.0);
        assertEquals(accelerationPSD, config2.getAccelerationPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config2.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config2.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config2.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config2.getRangeRateSD(), 0.0);
    }

    @Test
    public void testGetSetInitialPositionUncertainty() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialPositionUncertainty(initialPositionUncertainty);

        // check
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
    }

    @Test
    public void testGetSetDistanceInitialPositionUncertainty() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        final Distance initialPositionUncertainty1 = config.getDistanceInitialPositionUncertainty();
        assertEquals(0.0, initialPositionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialPositionUncertainty1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance initialPositionUncertainty2 = new Distance(initialPositionUncertainty, DistanceUnit.METER);
        config.setInitialPositionUncertainty(initialPositionUncertainty2);

        // check
        final Distance initialPositionUncertainty3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getDistanceInitialPositionUncertainty(initialPositionUncertainty3);
        final Distance initialPositionUncertainty4 = config.getDistanceInitialPositionUncertainty();

        assertEquals(initialPositionUncertainty2, initialPositionUncertainty3);
        assertEquals(initialPositionUncertainty2, initialPositionUncertainty4);
    }

    @Test
    public void testGetSetInitialVelocityUncertainty() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty);

        // check
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
    }

    @Test
    public void testGetSetSpeedInitialVelocityUncertainty() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        final Speed initialVelocityUncertainty1 = config.getSpeedInitialVelocityUncertainty();
        assertEquals(0.0,
                initialVelocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, initialVelocityUncertainty1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed initialVelocityUncertainty2 = new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty2);

        // check
        final Speed initialVelocityUncertainty3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getSpeedInitialVelocityUncertainty(initialVelocityUncertainty3);
        final Speed initialVelocityUncertainty4 = config.getSpeedInitialVelocityUncertainty();

        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty3);
        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty4);
    }

    @Test
    public void testGetSetInitialClockOffsetUncertainty() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getInitialClockOffsetUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialClockOffsetUncertainty(initialClockOffsetUncertainty);

        // check
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
    }

    @Test
    public void testGetSetDistanceInitialClockOffsetUncertainty() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        final Distance initialClockOffsetUncertainty1 = config.getDistanceInitialClockOffsetUncertainty();
        assertEquals(0.0, initialClockOffsetUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialClockOffsetUncertainty1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance initialClockOffsetUncertainty2 = new Distance(
                initialClockOffsetUncertainty, DistanceUnit.METER);
        config.setInitialClockOffsetUncertainty(initialClockOffsetUncertainty2);

        // check
        final Distance initialClockOffsetUncertainty3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getDistanceInitialClockOffsetUncertainty(initialClockOffsetUncertainty3);
        final Distance initialClockOffsetUncertainty4 = config.getDistanceInitialClockOffsetUncertainty();

        assertEquals(initialClockOffsetUncertainty2, initialClockOffsetUncertainty3);
        assertEquals(initialClockOffsetUncertainty2, initialClockOffsetUncertainty4);
    }

    @Test
    public void testGetSetInitialClockDriftUncertainty() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getInitialClockDriftUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialClockDriftUncertainty(initialClockDriftUncertainty);

        // check
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
    }

    @Test
    public void testGetSetSpeedInitialClockDriftUncertainty() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        final Speed initialClockDriftUncertainty1 = config.getSpeedInitialClockDriftUncertainty();
        assertEquals(0.0, initialClockDriftUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, initialClockDriftUncertainty1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed initialClockDriftUncertainty2 = new Speed(
                initialClockDriftUncertainty, SpeedUnit.METERS_PER_SECOND);
        config.setInitialClockDriftUncertainty(initialClockDriftUncertainty2);

        // check
        final Speed initialClockDriftUncertainty3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getSpeedInitialClockDriftUncertainty(initialClockDriftUncertainty3);
        final Speed initialClockDriftUncertainty4 = config.getSpeedInitialClockDriftUncertainty();

        assertEquals(initialClockDriftUncertainty2, initialClockDriftUncertainty3);
        assertEquals(initialClockDriftUncertainty2, initialClockDriftUncertainty4);
    }

    @Test
    public void testGetSetAccelerationPSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getAccelerationPSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setAccelerationPSD(accelerationPSD);

        // check
        assertEquals(accelerationPSD, config.getAccelerationPSD(), 0.0);
    }

    @Test
    public void testGetSetClockFrequencyPSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setClockFrequencyPSD(clockFrequencyPSD);

        // check
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
    }

    @Test
    public void testGetSetClockPhasePSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setClockPhasePSD(clockPhasePSD);

        // check
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
    }

    @Test
    public void testGetSetPseudoRangeSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setPseudoRangeSD(pseudoRangeSD);

        // check
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
    }

    @Test
    public void testGetSetDistancePseudoRangeSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        final Distance pseudoRangeSD1 = config.getDistancePseudoRangeSD();

        assertEquals(0.0, pseudoRangeSD1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, pseudoRangeSD1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance pseudoRangeSD2 = new Distance(pseudoRangeSD, DistanceUnit.METER);
        config.setPseudoRangeSD(pseudoRangeSD2);

        // check
        final Distance pseudoRangeSD3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getDistancePseudoRangeSD(pseudoRangeSD3);
        final Distance pseudoRangeSD4 = config.getDistancePseudoRangeSD();

        assertEquals(pseudoRangeSD2, pseudoRangeSD3);
        assertEquals(pseudoRangeSD2, pseudoRangeSD4);
    }

    @Test
    public void testGetSetRangeRateSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setRangeRateSD(rangeRateSD);

        // check
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    public void testGetSetSpeedRangeRateSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        final Speed rangeRateSD1 = config.getSpeedRangeRateSD();

        assertEquals(0.0, rangeRateSD1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, rangeRateSD1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Speed rangeRateSD2 = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);
        config.setRangeRateSD(rangeRateSD2);

        // check
        final Speed rangeRateSD3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getSpeedRangeRateSD(rangeRateSD3);
        final Speed rangeRateSD4 = config.getSpeedRangeRateSD();

        assertEquals(rangeRateSD2, rangeRateSD3);
        assertEquals(rangeRateSD2, rangeRateSD4);
    }

    @Test
    public void testSetValues() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default values
        assertEquals(0.0, config.getAccelerationPSD(), 0.0);
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setValues(initialPositionUncertainty, initialVelocityUncertainty, initialClockOffsetUncertainty,
                initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
                rangeRateSD);

        // check
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
        assertEquals(accelerationPSD, config.getAccelerationPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    public void testSetValues2() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default values
        assertEquals(0.0, config.getAccelerationPSD(), 0.0);
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance distancePseudoRangeSD = new Distance(pseudoRangeSD, DistanceUnit.METER);
        final Speed speedRangeRateSD = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        final Distance distanceInitialPositionUncertainty = new Distance(
                initialPositionUncertainty, DistanceUnit.METER);
        final Speed speedInitialVelocityUncertainty = new Speed(
                initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        final Distance distanceInitialClockOffsetUncertainty = new Distance(
                initialClockOffsetUncertainty, DistanceUnit.METER);
        final Speed speedInitialClockDriftUncertainty = new Speed(
                initialClockDriftUncertainty, SpeedUnit.METERS_PER_SECOND);

        config.setValues(distanceInitialPositionUncertainty, speedInitialVelocityUncertainty,
                distanceInitialClockOffsetUncertainty, speedInitialClockDriftUncertainty, accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, distancePseudoRangeSD, speedRangeRateSD);

        // check
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
        assertEquals(accelerationPSD, config.getAccelerationPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig();

        config1.copyTo(config2);

        // check
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config2.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config2.getInitialClockDriftUncertainty(), 0.0);
        assertEquals(accelerationPSD, config2.getAccelerationPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config2.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config2.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config2.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config2.getRangeRateSD(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config2.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config2.getInitialClockDriftUncertainty(), 0.0);
        assertEquals(accelerationPSD, config2.getAccelerationPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config2.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config2.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config2.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config2.getRangeRateSD(), 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config3 = new GNSSKalmanConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config3 = new GNSSKalmanConfig();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(config1.equals((Object)config1));
        //noinspection EqualsWithItself
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
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config3 = new GNSSKalmanConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);

        final Object config2 = config1.clone();

        assertEquals(config1, config2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(config1);
        final GNSSKalmanConfig config2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(config1, config2);
        assertNotSame(config1, config2);
    }
}
