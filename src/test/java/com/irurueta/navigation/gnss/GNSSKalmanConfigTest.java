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
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class GNSSKalmanConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var config = new GNSSKalmanConfig();

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
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

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
        final var distancePseudoRangeSD = new Distance(pseudoRangeSD, DistanceUnit.METER);
        final var speedRangeRateSD = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        final var distanceInitialPositionUncertainty = new Distance(initialPositionUncertainty, DistanceUnit.METER);
        final var speedInitialVelocityUncertainty = new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        final var distanceInitialClockOffsetUncertainty = new Distance(initialClockOffsetUncertainty,
                DistanceUnit.METER);
        final var speedInitialClockDriftUncertainty = new Speed(initialClockDriftUncertainty,
                SpeedUnit.METERS_PER_SECOND);

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
        final var config2 = new GNSSKalmanConfig(config);

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
    void testGetSetInitialPositionUncertainty() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialPositionUncertainty(initialPositionUncertainty);

        // check
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
    }

    @Test
    void testGetSetDistanceInitialPositionUncertainty() {
        final var config = new GNSSKalmanConfig();

        // check default value
        final var initialPositionUncertainty1 = config.getDistanceInitialPositionUncertainty();
        assertEquals(0.0, initialPositionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialPositionUncertainty1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var initialPositionUncertainty2 = new Distance(initialPositionUncertainty, DistanceUnit.METER);
        config.setInitialPositionUncertainty(initialPositionUncertainty2);

        // check
        final var initialPositionUncertainty3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getDistanceInitialPositionUncertainty(initialPositionUncertainty3);
        final var initialPositionUncertainty4 = config.getDistanceInitialPositionUncertainty();

        assertEquals(initialPositionUncertainty2, initialPositionUncertainty3);
        assertEquals(initialPositionUncertainty2, initialPositionUncertainty4);
    }

    @Test
    void testGetSetInitialVelocityUncertainty() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty);

        // check
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
    }

    @Test
    void testGetSetSpeedInitialVelocityUncertainty() {
        final var config = new GNSSKalmanConfig();

        // check default value
        final var initialVelocityUncertainty1 = config.getSpeedInitialVelocityUncertainty();
        assertEquals(0.0, initialVelocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, initialVelocityUncertainty1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var initialVelocityUncertainty2 = new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty2);

        // check
        final var initialVelocityUncertainty3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getSpeedInitialVelocityUncertainty(initialVelocityUncertainty3);
        final var initialVelocityUncertainty4 = config.getSpeedInitialVelocityUncertainty();

        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty3);
        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty4);
    }

    @Test
    void testGetSetInitialClockOffsetUncertainty() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getInitialClockOffsetUncertainty(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialClockOffsetUncertainty(initialClockOffsetUncertainty);

        // check
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
    }

    @Test
    void testGetSetDistanceInitialClockOffsetUncertainty() {
        final var config = new GNSSKalmanConfig();

        // check default value
        final var initialClockOffsetUncertainty1 = config.getDistanceInitialClockOffsetUncertainty();
        assertEquals(0.0, initialClockOffsetUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialClockOffsetUncertainty1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var initialClockOffsetUncertainty2 = new Distance(initialClockOffsetUncertainty, DistanceUnit.METER);
        config.setInitialClockOffsetUncertainty(initialClockOffsetUncertainty2);

        // check
        final var initialClockOffsetUncertainty3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getDistanceInitialClockOffsetUncertainty(initialClockOffsetUncertainty3);
        final var initialClockOffsetUncertainty4 = config.getDistanceInitialClockOffsetUncertainty();

        assertEquals(initialClockOffsetUncertainty2, initialClockOffsetUncertainty3);
        assertEquals(initialClockOffsetUncertainty2, initialClockOffsetUncertainty4);
    }

    @Test
    void testGetSetInitialClockDriftUncertainty() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getInitialClockDriftUncertainty(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialClockDriftUncertainty(initialClockDriftUncertainty);

        // check
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
    }

    @Test
    void testGetSetSpeedInitialClockDriftUncertainty() {
        final var config = new GNSSKalmanConfig();

        // check default value
        final var initialClockDriftUncertainty1 = config.getSpeedInitialClockDriftUncertainty();
        assertEquals(0.0, initialClockDriftUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, initialClockDriftUncertainty1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var initialClockDriftUncertainty2 = new Speed(initialClockDriftUncertainty, SpeedUnit.METERS_PER_SECOND);
        config.setInitialClockDriftUncertainty(initialClockDriftUncertainty2);

        // check
        final var initialClockDriftUncertainty3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getSpeedInitialClockDriftUncertainty(initialClockDriftUncertainty3);
        final var initialClockDriftUncertainty4 = config.getSpeedInitialClockDriftUncertainty();

        assertEquals(initialClockDriftUncertainty2, initialClockDriftUncertainty3);
        assertEquals(initialClockDriftUncertainty2, initialClockDriftUncertainty4);
    }

    @Test
    void testGetSetAccelerationPSD() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getAccelerationPSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setAccelerationPSD(accelerationPSD);

        // check
        assertEquals(accelerationPSD, config.getAccelerationPSD(), 0.0);
    }

    @Test
    void testGetSetClockFrequencyPSD() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setClockFrequencyPSD(clockFrequencyPSD);

        // check
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
    }

    @Test
    void testGetSetClockPhasePSD() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setClockPhasePSD(clockPhasePSD);

        // check
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
    }

    @Test
    void testGetSetPseudoRangeSD() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setPseudoRangeSD(pseudoRangeSD);

        // check
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
    }

    @Test
    void testGetSetDistancePseudoRangeSD() {
        final var config = new GNSSKalmanConfig();

        // check default value
        final var pseudoRangeSD1 = config.getDistancePseudoRangeSD();

        assertEquals(0.0, pseudoRangeSD1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, pseudoRangeSD1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var pseudoRangeSD2 = new Distance(pseudoRangeSD, DistanceUnit.METER);
        config.setPseudoRangeSD(pseudoRangeSD2);

        // check
        final var pseudoRangeSD3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getDistancePseudoRangeSD(pseudoRangeSD3);
        final var pseudoRangeSD4 = config.getDistancePseudoRangeSD();

        assertEquals(pseudoRangeSD2, pseudoRangeSD3);
        assertEquals(pseudoRangeSD2, pseudoRangeSD4);
    }

    @Test
    void testGetSetRangeRateSD() {
        final var config = new GNSSKalmanConfig();

        // check default value
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setRangeRateSD(rangeRateSD);

        // check
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    void testGetSetSpeedRangeRateSD() {
        final var config = new GNSSKalmanConfig();

        // check default value
        final var rangeRateSD1 = config.getSpeedRangeRateSD();

        assertEquals(0.0, rangeRateSD1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, rangeRateSD1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD2 = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);
        config.setRangeRateSD(rangeRateSD2);

        // check
        final var rangeRateSD3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getSpeedRangeRateSD(rangeRateSD3);
        final var rangeRateSD4 = config.getSpeedRangeRateSD();

        assertEquals(rangeRateSD2, rangeRateSD3);
        assertEquals(rangeRateSD2, rangeRateSD4);
    }

    @Test
    void testSetValues() {
        final var config = new GNSSKalmanConfig();

        // check default values
        assertEquals(0.0, config.getAccelerationPSD(), 0.0);
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set values
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

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
    void testSetValues2() {
        final var config = new GNSSKalmanConfig();

        // check default values
        assertEquals(0.0, config.getAccelerationPSD(), 0.0);
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set values
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var distancePseudoRangeSD = new Distance(pseudoRangeSD, DistanceUnit.METER);
        final var speedRangeRateSD = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        final var distanceInitialPositionUncertainty = new Distance(initialPositionUncertainty, DistanceUnit.METER);
        final var speedInitialVelocityUncertainty = new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        final var distanceInitialClockOffsetUncertainty = new Distance(initialClockOffsetUncertainty,
                DistanceUnit.METER);
        final var speedInitialClockDriftUncertainty = new Speed(initialClockDriftUncertainty,
                SpeedUnit.METERS_PER_SECOND);

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
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new GNSSKalmanConfig();

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
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new GNSSKalmanConfig();

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
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config3 = new GNSSKalmanConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config3 = new GNSSKalmanConfig();

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
        assertNotEquals(new Object(), config1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config3 = new GNSSKalmanConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);

        final var config2 = config1.clone();

        assertEquals(config1, config2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(config1);
        final var config2 = SerializationHelper.<GNSSKalmanConfig>deserialize(bytes);

        // check
        assertEquals(config1, config2);
        assertNotSame(config1, config2);
    }
}
