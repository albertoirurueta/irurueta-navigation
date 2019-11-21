package com.irurueta.navigation.gnss;

import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

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
        assertEquals(config.getAccelerationPSD(), 0.0, 0.0);
        assertEquals(config.getClockFrequencyPSD(), 0.0, 0.0);
        assertEquals(config.getClockPhasePSD(), 0.0, 0.0);
        assertEquals(config.getPseudoRangeSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateSD(), 0.0, 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config = new GNSSKalmanConfig(accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);

        // check default values
        assertEquals(config.getAccelerationPSD(), accelerationPSD, 0.0);
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);


        //test other constructor with values
        final Distance distancePseudoRangeSD = new Distance(pseudoRangeSD,
                DistanceUnit.METER);
        final Speed speedRangeRateSD = new Speed(rangeRateSD,
                SpeedUnit.METERS_PER_SECOND);

        config = new GNSSKalmanConfig(accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, distancePseudoRangeSD, speedRangeRateSD);

        // check default values
        assertEquals(config.getAccelerationPSD(), accelerationPSD, 0.0);
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);


        // test copy constructor
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig(config);

        // check default values
        assertEquals(config2.getAccelerationPSD(), accelerationPSD, 0.0);
        assertEquals(config2.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config2.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config2.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config2.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testGetSetAccelerationPSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(config.getAccelerationPSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setAccelerationPSD(accelerationPSD);

        // check
        assertEquals(config.getAccelerationPSD(), accelerationPSD, 0.0);
    }

    @Test
    public void testGetSetClockFrequencyPSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(config.getClockFrequencyPSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setClockFrequencyPSD(clockFrequencyPSD);

        // check
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
    }

    @Test
    public void testGetSetClockPhasePSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(config.getClockPhasePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setClockPhasePSD(clockPhasePSD);

        // check
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
    }

    @Test
    public void testGetSetPseudoRangeSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        assertEquals(config.getPseudoRangeSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setPseudoRangeSD(pseudoRangeSD);

        // check
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
    }

    @Test
    public void testGetSetDistancePseudoRangeSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        final Distance pseudoRangeSD1 = config.getDistancePseudoRangeSD();

        assertEquals(pseudoRangeSD1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(pseudoRangeSD1.getUnit(), DistanceUnit.METER);

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
        assertEquals(config.getRangeRateSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setRangeRateSD(rangeRateSD);

        // check
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testGetSetSpeedRangeRateSD() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default value
        final Speed rangeRateSD1 = config.getSpeedRangeRateSD();

        assertEquals(rangeRateSD1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(rangeRateSD1.getUnit(), SpeedUnit.METERS_PER_SECOND);

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
        assertEquals(config.getAccelerationPSD(), 0.0, 0.0);
        assertEquals(config.getClockFrequencyPSD(), 0.0, 0.0);
        assertEquals(config.getClockPhasePSD(), 0.0, 0.0);
        assertEquals(config.getPseudoRangeSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateSD(), 0.0, 0.0);

        // set values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setValues(accelerationPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);

        // check
        assertEquals(config.getAccelerationPSD(), accelerationPSD, 0.0);
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testSetValues2() {
        final GNSSKalmanConfig config = new GNSSKalmanConfig();

        // check default values
        assertEquals(config.getAccelerationPSD(), 0.0, 0.0);
        assertEquals(config.getClockFrequencyPSD(), 0.0, 0.0);
        assertEquals(config.getClockPhasePSD(), 0.0, 0.0);
        assertEquals(config.getPseudoRangeSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateSD(), 0.0, 0.0);

        // set values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance distancePseudoRangeSD = new Distance(pseudoRangeSD,
                DistanceUnit.METER);
        final Speed speedRangeRateSD = new Speed(rangeRateSD,
                SpeedUnit.METERS_PER_SECOND);

        config.setValues(accelerationPSD, clockFrequencyPSD, clockPhasePSD,
                distancePseudoRangeSD, speedRangeRateSD);

        // check
        assertEquals(config.getAccelerationPSD(), accelerationPSD, 0.0);
        assertEquals(config.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig();

        config1.copyTo(config2);

        // check
        assertEquals(config2.getAccelerationPSD(), accelerationPSD, 0.0);
        assertEquals(config2.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config2.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config2.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config2.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(config2.getAccelerationPSD(), accelerationPSD, 0.0);
        assertEquals(config2.getClockFrequencyPSD(), clockFrequencyPSD, 0.0);
        assertEquals(config2.getClockPhasePSD(), clockPhasePSD, 0.0);
        assertEquals(config2.getPseudoRangeSD(), pseudoRangeSD, 0.0);
        assertEquals(config2.getRangeRateSD(), rangeRateSD, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config3 = new GNSSKalmanConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config3 = new GNSSKalmanConfig();

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
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final GNSSKalmanConfig config3 = new GNSSKalmanConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final GNSSKalmanConfig config1 = new GNSSKalmanConfig(accelerationPSD,
                clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);

        final Object config2 = config1.clone();

        assertEquals(config1, config2);
    }
}
