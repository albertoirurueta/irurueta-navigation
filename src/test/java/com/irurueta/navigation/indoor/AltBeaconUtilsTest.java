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
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

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

    public AltBeaconUtilsTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        //noinspection all
        assertNotNull(new AltBeaconUtils());
    }

    @Test
    public void testGetK() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);

        double k = AltBeaconUtils.getK(frequency);

        assertEquals(frequency, AltBeaconUtils.getFrequency(k), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetKWithCoefficients() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        double pathLossExponent = randomizer.nextDouble(MIN_PATHLOSS, MAX_PATHLOSS);

        double coefficient1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency,
                pathLossExponent);
        double coefficient2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        double k = AltBeaconUtils.getK(coefficient1, coefficient2);

        assertEquals(frequency, AltBeaconUtils.getFrequency(k), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetFrequency() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double k = randomizer.nextDouble(MIN_K, MAX_K);


        double frequency = AltBeaconUtils.getFrequency(k);

        assertEquals(k, AltBeaconUtils.getK(frequency), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetFrequencyWithCoefficients() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);

        double frequency = AltBeaconUtils.getFrequency(c1, c2);
        double n = AltBeaconUtils.getPathLossExponent(c2);

        assertEquals(c1, AltBeaconUtils.getCoefficient1WithFrequency(frequency, n),
                ABSOLUTE_ERROR);
        assertEquals(c2, AltBeaconUtils.getCoefficient2(n), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetPathLossExponent() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);

        double n = AltBeaconUtils.getPathLossExponent(c2);

        assertEquals(c2, AltBeaconUtils.getCoefficient2(n), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetDistance() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        double ratio = randomizer.nextDouble(MIN_RATIO, MAX_RATIO);

        double distance = AltBeaconUtils.getDistance(c1, c2, c3, ratio);

        assertEquals(ratio, AltBeaconUtils.getRatio(c1, c2, c3, distance),
                ABSOLUTE_ERROR);
        assertEquals(distance, c1 * Math.pow(ratio, c2) + c3,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetDistanceWithPower() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        double ratio = randomizer.nextDouble(MIN_RATIO, MAX_RATIO);

        double receivedPower = AltBeaconUtils.getReceivedPower(ratio,
                transmittedPower);

        double distance = AltBeaconUtils.getDistance(c1, c2, c3,
                receivedPower, transmittedPower);

        assertEquals(ratio, AltBeaconUtils.getRatio(c1, c2, c3, distance),
                ABSOLUTE_ERROR);
        assertEquals(distance, c1 * Math.pow(ratio, c2) + c3,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRatio() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

        double ratio = AltBeaconUtils.getRatio(c1, c2, c3, distance);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3, ratio),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRatioWithPower() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        double ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        //check
        assertEquals(ratio, receivedPower / transmittedPower,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetReceivedPower() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        double ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        assertEquals(receivedPower,
                AltBeaconUtils.getReceivedPower(ratio, transmittedPower),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetReceivedPowerWithCoefficients() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
        double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        double receivedPower = AltBeaconUtils.getReceivedPower(c1, c2, c3,
                distance, transmittedPower);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3,
                receivedPower, transmittedPower), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetTransmittedPower() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);
        double transmittedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        double ratio = AltBeaconUtils.getRatio(receivedPower, transmittedPower);

        assertEquals(transmittedPower,
                AltBeaconUtils.getTransmittedPower(ratio, receivedPower),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetTransmittedPowerWithCoefficients() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double c1 = randomizer.nextDouble(MIN_C1, MAX_C1);
        double c2 = randomizer.nextDouble(MIN_C2, MAX_C2);
        double c3 = randomizer.nextDouble(MIN_C3, MAX_C3);

        double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
        double receivedPower = randomizer.nextDouble(MIN_POWER, MAX_POWER);

        double transmittedPower = AltBeaconUtils.getTransmittedPower(c1, c2, c3,
                distance, receivedPower);

        assertEquals(distance, AltBeaconUtils.getDistance(c1, c2, c3,
                receivedPower, transmittedPower), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetCoefficient1() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        double pathLossExponent = randomizer.nextDouble(MIN_PATHLOSS, MAX_PATHLOSS);

        double coefficient1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency,
                pathLossExponent);
        double coefficient2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        double k = AltBeaconUtils.getK(coefficient1, coefficient2);

        assertEquals(coefficient1, AltBeaconUtils.getCoefficient1(k, pathLossExponent),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetCoefficient1WithFrequency() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double frequency = randomizer.nextDouble(MIN_FREQUENCY, MAX_FREQUENCY);
        double pathLossExponent = randomizer.nextDouble(MIN_PATHLOSS, MAX_PATHLOSS);

        double c1 = AltBeaconUtils.getCoefficient1WithFrequency(frequency,
                pathLossExponent);
        double c2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        assertEquals(frequency, AltBeaconUtils.getFrequency(c1, c2),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetCoefficient2() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double pathLossExponent = randomizer.nextDouble(MIN_PATHLOSS, MAX_PATHLOSS);

        double c2 = AltBeaconUtils.getCoefficient2(pathLossExponent);

        assertEquals(pathLossExponent, AltBeaconUtils.getPathLossExponent(c2),
                ABSOLUTE_ERROR);
    }
}
