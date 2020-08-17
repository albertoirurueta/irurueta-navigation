/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.indoor;

import com.irurueta.statistics.UniformRandomizer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class AltBeaconUtilsTest {

    private static final double MIN_FREQUENCY = 2.4e9;
    private static final double MAX_FREQUENCY = 2.45e9;

    private static final double MIN_PATHLOSS = 1.6;
    private static final double MAX_PATHLOSS = 2.0;

    private static final double MIN_K = 0.5;
    private static final double MAX_K = 0.6;

    private static final double MIN_C1 = 0.08;
    private static final double MAX_C1 = 0.09;

    private static final double MIN_C2 = -0.6;
    private static final double MAX_C2 = -0.5;

    private static final double MIN_C3 = 0.0;
    private static final double MAX_C3 = 1.0;

    private static final double MIN_RATIO = 0.5;
    private static final double MAX_RATIO = 1.0;

    private static final double MIN_POWER = 0.5;
    private static final double MAX_POWER = 1.0;

    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 5.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    public AltBeaconUtilsTest() {
    }

    @BeforeClass
    public static void setUpClass() {
    }

    @AfterClass
    public static void tearDownClass() {
    }

    @Before
    public void setUp() {
    }

    @After
    public void tearDown() {
    }

    @Test
    public void testGetK() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);

        final double k = AltBeaconUtils.getK(frequency);

        assertEquals(frequency, AltBeaconUtils.getFrequency(k), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetKWithCoefficients() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        final double pathLossExponent = randomizer.nextDouble(MIN_PATHLOSS, MAX_PATHLOSS);

        final double coefficient1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency,
                pathLossExponent);
        final double coefficient2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        final double k = AltBeaconUtils.getK(coefficient1, coefficient2);

        assertEquals(frequency / 1e9,
                AltBeaconUtils.getFrequency(k) / 1e9, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetFrequency() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double k = randomizer.nextDouble(MIN_K, MAX_K);


        final double frequency = AltBeaconUtils.getFrequency(k);

        assertEquals(k, AltBeaconUtils.getK(frequency), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetFrequencyWithCoefficients() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);

        final double frequency = AltBeaconUtils.getFrequency(c1, c2);
        final double n = AltBeaconUtils.getPathLossExponent(c2);

        assertEquals(c1, AltBeaconUtils.getCoefficient1WithFrequency(frequency, n),
                ABSOLUTE_ERROR);
        assertEquals(c2, AltBeaconUtils.getCoefficient2(n), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetPathLossExponent() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);

        final double n = AltBeaconUtils.getPathLossExponent(c2);

        assertEquals(c2, AltBeaconUtils.getCoefficient2(n), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetDistance() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final double ratio = randomizer.nextDouble(MIN_RATIO, MAX_RATIO);

        final double distance = AltBeaconUtils.getDistance(c1, c2, c3, ratio);

        assertEquals(ratio, AltBeaconUtils.getRatio(c1, c2, c3, distance),
                ABSOLUTE_ERROR);
        assertEquals(distance, c1 * Math.pow(ratio, c2) + c3,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetDistanceWithPower() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        final double ratio = randomizer.nextDouble(MIN_RATIO, MAX_RATIO);

        final double receivedPower = AltBeaconUtils.getReceivedPower(ratio,
                transmittedPower);

        final double distance = AltBeaconUtils.getDistance(c1, c2, c3,
                receivedPower, transmittedPower);

        assertEquals(ratio, AltBeaconUtils.getRatio(c1, c2, c3, distance),
                ABSOLUTE_ERROR);
        assertEquals(distance, c1 * Math.pow(ratio, c2) + c3,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRatio() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

        final double ratio = AltBeaconUtils.getRatio(c1, c2, c3, distance);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3, ratio),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRatioWithPower() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        final double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final double ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        //check
        assertEquals(ratio, receivedPower / transmittedPower,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetReceivedPower() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        final double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final double ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        assertEquals(receivedPower,
                AltBeaconUtils.getReceivedPower(ratio, transmittedPower),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetReceivedPowerWithCoefficients() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
        final double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final double receivedPower = AltBeaconUtils.getReceivedPower(c1, c2, c3,
                distance, transmittedPower);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3,
                receivedPower, transmittedPower), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetTransmittedPower() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        final double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final double ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        assertEquals(transmittedPower,
                AltBeaconUtils.getTransmittedPower(ratio, receivedPower),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetTransmittedPowerWithCoefficients() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        final double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        final double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
        final double receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        final double transmittedPower = AltBeaconUtils.getTransmittedPower(c1, c2, c3,
                distance, receivedPower);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3,
                receivedPower, transmittedPower), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetCoefficient1() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        final double pathLossExponent = randomizer.nextDouble(MIN_PATHLOSS, MAX_PATHLOSS);

        final double coefficient1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency,
                pathLossExponent);
        final double coefficient2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        final double k = AltBeaconUtils.getK(coefficient1, coefficient2);

        assertEquals(coefficient1, AltBeaconUtils.getCoefficient1(k, pathLossExponent),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetCoefficient1WithFrequency() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        final double pathLossExponent = randomizer.nextDouble(MIN_PATHLOSS, MAX_PATHLOSS);

        final double c1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency,
                pathLossExponent);
        final double c2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        assertEquals(frequency, AltBeaconUtils.getFrequency(c1, c2),
                10.0 * ABSOLUTE_ERROR);
    }

    @Test
    public void testGetCoefficient2() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double pathLossExponent = randomizer.nextDouble(MIN_PATHLOSS, MAX_PATHLOSS);

        final double c2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        assertEquals(pathLossExponent, AltBeaconUtils.getPathLossExponent(c2),
                ABSOLUTE_ERROR);
    }
}
