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
package com.irurueta.navigation.indoor.oldposition;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class PositionEstimatorHelperTest {

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double FALLBACK_DISTANCE_STANDARD_DEVIATION = 1e-3;

    private static final double TX_POWER_VARIANCE = 0.1;
    private static final double RX_POWER_VARIANCE = 0.5;
    private static final double PATHLOSS_EXPONENT_VARIANCE = 0.001;

    private static final double POSITION_VARIANCE = 0.01;

    public PositionEstimatorHelperTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testBuildPositionsAndDistancesRssiReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm, pathLossExponent,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint,
                    positions, distances);

            //check that positions and distances are not modified if no sources or
            // fingerprint are provided
            PositionEstimatorHelper.buildPositionsAndDistances(
                    null, null, positions, distances);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsAndDistancesRangingReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm, pathLossExponent,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint,
                    positions, distances);

            //check that positions and distances are not modified if no sources or
            // fingerprint are provided
            PositionEstimatorHelper.buildPositionsAndDistances(
                    null, null, positions, distances);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertEquals(distances.get(i), readings.get(i).getDistance(), 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsAndDistancesRangingAndRssiReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm, pathLossExponent,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint,
                    positions, distances);

            //check that positions and distances are not modified if no sources or
            // fingerprint are provided
            PositionEstimatorHelper.buildPositionsAndDistances(
                    null, null, positions, distances);

            //check
            assertEquals(positions.size(), 2*numSources);
            assertEquals(distances.size(), 2*numSources);

            for (int i = 0, j = 0; i < numSources; i++, j += 2) {
                assertEquals(sources.get(i).getPosition(), positions.get(j));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 1));
                assertEquals(distances.get(j), distances.get(j + 1), ABSOLUTE_ERROR);
                assertEquals(distances.get(j), readings.get(i).getDistance(), 0.0);
                assertEquals(distances.get(j + 1), readings.get(i).getDistance(), ABSOLUTE_ERROR);
            }
        }
    }

    @Test
    public void testBuildPositionsAndDistancesNonRadioSourceWithPower() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointLocated2D locatedAccessPoint =
                        new WifiAccessPointLocated2D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsAndDistances(sources, fingerprint,
                    positions, distances);

            //check
            assertTrue(positions.isEmpty());
            assertTrue(distances.isEmpty());
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsRssiReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);
            assertEquals(distanceStandardDeviations.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsRangingReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);
            assertEquals(distanceStandardDeviations.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertEquals(distances.get(i), readings.get(i).getDistance(), 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsRangingAndRssiReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RangingAndRssiReading<>(accessPoint, distance, rssi,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                    new RangingAndRssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), 2*numSources);
            assertEquals(distances.size(), 2*numSources);
            assertEquals(distanceStandardDeviations.size(), 2*numSources);

            for (int i = 0, j = 0; i < numSources; i++, j += 2) {
                assertEquals(sources.get(i).getPosition(), positions.get(j));
                assertEquals(sources.get(i).getPosition(), positions.get(j + 1));

                assertEquals(distances.get(j), distances.get(j + 1), ABSOLUTE_ERROR);
                assertEquals(distances.get(j), readings.get(i).getDistance(), 0.0);
                assertEquals(distances.get(j + 1), readings.get(i).getDistance(), ABSOLUTE_ERROR);

                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsNoRadioSourceWithPower() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointLocated2D locatedAccessPoint =
                        new WifiAccessPointLocated2D(bssid, FREQUENCY,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertTrue(positions.isEmpty());
            assertTrue(distances.isEmpty());
            assertTrue(distanceStandardDeviations.isEmpty());
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsForceIllegalArgumentException() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

        InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        double pathLossExponent = randomizer.nextDouble(
                MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

        List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        for (int i = 0; i < numSources; i++) {
            InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
            String bssid = String.valueOf(i);

            WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                    new WifiAccessPointWithPowerAndLocated2D(bssid,
                            FREQUENCY, transmittedPowerdBm,
                            Math.sqrt(TX_POWER_VARIANCE),
                            pathLossExponent,
                            Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                            accessPointPosition);
            sources.add(locatedAccessPoint);

            WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

            double distance = position.distanceTo(accessPointPosition);

            double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                    distance, pathLossExponent));

            readings.add(new RssiReading<>(accessPoint, rssi,
                    Math.sqrt(RX_POWER_VARIANCE)));
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>(readings);

        List<Point2D> positions = new ArrayList<>();
        List<Double> distances = new ArrayList<>();
        List<Double> distanceStandardDeviations = new ArrayList<>();
        try {
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    -FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsRssiReadingsWithPositionCovariance() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                Matrix positionCovariance = Matrix.diagonal(
                        new double[]{POSITION_VARIANCE, POSITION_VARIANCE});

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition, positionCovariance);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);
            assertEquals(distanceStandardDeviations.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsRangingReadingsWithPositionCovariance() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                Matrix positionCovariance = Matrix.diagonal(
                        new double[]{POSITION_VARIANCE, POSITION_VARIANCE});

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition, positionCovariance);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance,
                        FALLBACK_DISTANCE_STANDARD_DEVIATION));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);
            assertEquals(distanceStandardDeviations.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertEquals(distances.get(i), readings.get(i).getDistance(), 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsRssiReadingsNoVariances() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm, pathLossExponent,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);
            assertEquals(distanceStandardDeviations.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertEquals(distanceStandardDeviations.get(i),
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsRangingReadingsNoVariances() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm, pathLossExponent,
                                accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);
            assertEquals(distanceStandardDeviations.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertEquals(distances.get(i), readings.get(i).getDistance(), 0.0);
                assertEquals(distanceStandardDeviations.get(i),
                        FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesAndDistanceStandardDeviationsInvalidPositionCovariance() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                Matrix positionCovariance = Matrix.diagonal(
                        new double[]{Double.NaN, Double.NaN});

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition, positionCovariance);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    sources, fingerprint, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);
            assertEquals(distanceStandardDeviations.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
            }
        }
    }

    @Test
    public void testBuildPositionsDistancesDistanceStandardDeviationsAndQualityScoresRssiReadings() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            List<WifiAccessPointWithPowerAndLocated2D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            double[] qualityScores = new double[numSources];
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint2D accessPointPosition = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated2D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(bssid,
                                FREQUENCY, transmittedPowerdBm,
                                Math.sqrt(TX_POWER_VARIANCE),
                                pathLossExponent,
                                Math.sqrt(PATHLOSS_EXPONENT_VARIANCE),
                                accessPointPosition);
                sources.add(locatedAccessPoint);
                qualityScores[i] = randomizer.nextDouble();

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double rssi = Utils.powerTodBm(receivedPower(transmittedPower,
                        distance, pathLossExponent));

                readings.add(new RssiReading<>(accessPoint, rssi,
                        Math.sqrt(RX_POWER_VARIANCE)));
            }

            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<Point2D> positions = new ArrayList<>();
            List<Double> distances = new ArrayList<>();
            List<Double> distanceStandardDeviations = new ArrayList<>();
            List<Double> distanceQualityScores = new ArrayList<>();
            PositionEstimatorHelper.buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
                    sources, fingerprint, qualityScores, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations, distanceQualityScores);

            //check that positions, distances and distance standard deviations are not
            //modified if no sources or fingerprint are provided
            PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                    null, null, true,
                    FALLBACK_DISTANCE_STANDARD_DEVIATION, positions, distances,
                    distanceStandardDeviations);

            //check
            assertEquals(positions.size(), numSources);
            assertEquals(distances.size(), numSources);
            assertEquals(distanceStandardDeviations.size(), numSources);

            for (int i = 0; i < numSources; i++) {
                assertEquals(sources.get(i).getPosition(), positions.get(i));
                assertTrue(distances.get(i) > 0.0);
                assertTrue(distanceStandardDeviations.get(i) > 0.0);
                assertEquals(qualityScores[i], distanceQualityScores.get(i), 0.0);
            }
        }
    }


    private double receivedPower(double equivalentTransmittedPower,
                                 double distance, double pathLossExponent) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }
}
