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
import static org.junit.Assert.assertFalse;

public class INSLooselyCoupledKalmanConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

        // check default values
        assertEquals(config.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerBiasPSD(), 0.0, 0.0);
        assertEquals(config.getGyroBiasPSD(), 0.0, 0.0);
        assertEquals(config.getPositionNoiseSD(), 0.0, 0.0);
        assertEquals(config.getVelocityNoiseSD(), 0.0, 0.0);

        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config = new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);

        // check default values
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD,
                0.0);
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config.getPositionNoiseSD(), positionNoiseSD, 0.0);
        assertEquals(config.getVelocityNoiseSD(), velocityNoiseSD, 0.0);


        // test constructor with values
        final Distance positionNoiseSDDistance = new Distance(positionNoiseSD,
                DistanceUnit.METER);
        final Speed velocityNoiseSDSpeed = new Speed(velocityNoiseSD,
                SpeedUnit.METERS_PER_SECOND);

        config = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD,
                positionNoiseSDDistance, velocityNoiseSDSpeed);

        // check default values
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD,
                0.0);
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config.getPositionNoiseSD(), positionNoiseSD, 0.0);
        assertEquals(config.getVelocityNoiseSD(), velocityNoiseSD, 0.0);
    }

    @Test
    public void testGetSetGyroNoisePSD() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getGyroNoisePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setGyroNoisePSD(gyroNoisePSD);

        // check
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
    }

    @Test
    public void testGetSetAccelerometerNoisePSD() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getAccelerometerNoisePSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerometerNoisePSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        config.setAccelerometerNoisePSD(accelerometerNoisePSD);

        // check
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasPSD() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getAccelerometerBiasPSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerometerBiasPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        config.setAccelerometerBiasPSD(accelerometerBiasPSD);

        // check
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD,
                0.0);
    }

    @Test
    public void testGetSetGyroBiasPSD() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getGyroBiasPSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setGyroBiasPSD(gyroBiasPSD);

        // check
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
    }

    @Test
    public void testGetSetPositionNoiseSD() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getPositionNoiseSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setPositionNoiseSD(positionNoiseSD);

        // check
        assertEquals(config.getPositionNoiseSD(), positionNoiseSD, 0.0);
    }

    @Test
    public void testGetSetVelocityNoiseSD() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default value
        assertEquals(config.getVelocityNoiseSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setVelocityNoiseSD(velocityNoiseSD);

        // check
        assertEquals(config.getVelocityNoiseSD(), velocityNoiseSD, 0.0);
    }

    @Test
    public void testSetValues() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default values
        assertEquals(config.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerBiasPSD(), 0.0, 0.0);
        assertEquals(config.getGyroBiasPSD(), 0.0, 0.0);
        assertEquals(config.getPositionNoiseSD(), 0.0, 0.0);
        assertEquals(config.getVelocityNoiseSD(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, positionNoiseSD, velocityNoiseSD);

        // check
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD,
                0.0);
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config.getPositionNoiseSD(), positionNoiseSD, 0.0);
        assertEquals(config.getVelocityNoiseSD(), velocityNoiseSD, 0.0);
    }

    @Test
    public void testGetSetPositionNoiseSDAsDistance() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default value
        final Distance positionNoise1 = config.getPositionNoiseSDAsDistance();

        assertEquals(positionNoise1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(positionNoise1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance positionNoise2 = new Distance(positionNoiseSD,
                DistanceUnit.METER);

        config.setPositionNoiseSD(positionNoise2);

        // check
        final Distance positionNoise3 =
                new Distance(0.0, DistanceUnit.KILOMETER);
        config.getPositionNoiseSDAsDistance(positionNoise3);
        final Distance positionNoise4 = config.getPositionNoiseSDAsDistance();

        assertEquals(positionNoise2, positionNoise3);
        assertEquals(positionNoise2, positionNoise4);
    }

    @Test
    public void testGetSetVelocityNoiseSDAsSpeed() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default value
        final Speed velocityNoise1 = config.getVelocityNoiseSDAsSpeed();

        assertEquals(velocityNoise1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(velocityNoise1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Speed velocityNoise2 = new Speed(velocityNoiseSD,
                SpeedUnit.METERS_PER_SECOND);

        config.setVelocityNoiseSD(velocityNoise2);

        // check
        final Speed velocityNoise3 =
                new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getVelocityNoiseSDAsSpeed(velocityNoise3);
        final Speed velocityNoise4 = config.getVelocityNoiseSDAsSpeed();

        assertEquals(velocityNoise2, velocityNoise3);
        assertEquals(velocityNoise2, velocityNoise4);
    }

    @Test
    public void testSetValues2() {
        final INSLooselyCoupledKalmanConfig config =
                new INSLooselyCoupledKalmanConfig();

        // check default values
        assertEquals(config.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(config.getAccelerometerBiasPSD(), 0.0, 0.0);
        assertEquals(config.getGyroBiasPSD(), 0.0, 0.0);
        assertEquals(config.getPositionNoiseSD(), 0.0, 0.0);
        assertEquals(config.getVelocityNoiseSD(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance positionNoiseSDDistance = new Distance(positionNoiseSD,
                DistanceUnit.METER);
        final Speed velocityNoiseSDSpeed = new Speed(velocityNoiseSD,
                SpeedUnit.METERS_PER_SECOND);

        config.setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, positionNoiseSDDistance, velocityNoiseSDSpeed);

        // check
        assertEquals(config.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config.getAccelerometerBiasPSD(), accelerometerBiasPSD,
                0.0);
        assertEquals(config.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config.getPositionNoiseSD(), positionNoiseSD, 0.0);
        assertEquals(config.getVelocityNoiseSD(), velocityNoiseSD, 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 =
                new INSLooselyCoupledKalmanConfig();

        config1.copyTo(config2);

        // check
        assertEquals(config2.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config2.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config2.getAccelerometerBiasPSD(), accelerometerBiasPSD,
                0.0);
        assertEquals(config2.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config2.getPositionNoiseSD(), positionNoiseSD, 0.0);
        assertEquals(config2.getVelocityNoiseSD(), velocityNoiseSD, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 =
                new INSLooselyCoupledKalmanConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(config2.getGyroNoisePSD(), gyroNoisePSD, 0.0);
        assertEquals(config2.getAccelerometerNoisePSD(), accelerometerNoisePSD,
                0.0);
        assertEquals(config2.getAccelerometerBiasPSD(), accelerometerBiasPSD,
                0.0);
        assertEquals(config2.getGyroBiasPSD(), gyroBiasPSD, 0.0);
        assertEquals(config2.getPositionNoiseSD(), positionNoiseSD, 0.0);
        assertEquals(config2.getVelocityNoiseSD(), velocityNoiseSD, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config3 =
                new INSLooselyCoupledKalmanConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config3 =
                new INSLooselyCoupledKalmanConfig();

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
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config3 =
                new INSLooselyCoupledKalmanConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD =
                randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 =
                new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                        accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);

        final Object config2 = config1.clone();

        assertEquals(config1, config2);
    }
}
