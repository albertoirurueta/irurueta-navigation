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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.KDTree2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;
import static com.irurueta.navigation.indoor.Utils.*;

public class WifiKNearestFinderTest {

    private static final Logger LOGGER = Logger.getLogger(
            WifiKNearestFinderTest.class.getName());

    private static final int MIN_RSSI = -100;
    private static final int MAX_RSSI = -50;

    private static final int MIN_AP = 1;
    private static final int MAX_AP = 5;

    private static final int MIN_FINGERPRINTS = 50;
    private static final int MAX_FINGERPRINTS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double SEPARATION_POS = 1.0;

    private static final double ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final double SPEED_OF_LIGHT = 3e8; //(m/s)

    private static final int MAX_K = 20;

    public WifiKNearestFinderTest() { }

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
        List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                new ArrayList<>();
        WifiKNearestFinder<Point2D> finder = new WifiKNearestFinder<>(
                fingerprints);

        //check
        assertSame(finder.getFingerprints(), fingerprints);

        //Force IllegalArgumentException
        finder = null;
        try {
            finder = new WifiKNearestFinder<>(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(finder);
    }

    @Test
    public void testFindNearestTo() {
        int numValid = 0;
        double avgValidSignalDistance = 0.0;
        double avgValidDistance = 0.0;
        double avgSignalDistance = 0.0;
        double avgDistance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find closest fingerprint
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint1 =
                     finder.findNearestTo(fingerprint);
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint2 =
                    WifiKNearestFinder.findNearestTo(fingerprint, fingerprints);

            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints1 =
                    finder.findKNearestTo(fingerprint, 1);
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints2 =
                    new ArrayList<>();
            List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints2,
                    nearestSqrDistances);

            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint3 =
                    closestFingerprints1.get(0);
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint4 =
                    closestFingerprints2.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint1.getPosition().distanceTo(position);

            Point2D nearestPosition = tree.nearestPoint(position);
            if (!nearestPosition.equals(
                    closestFingerprint1.getPosition(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(nearestPosition.equals(
                    closestFingerprint1.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(
                    closestFingerprint2.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(
                    closestFingerprint3.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(
                    closestFingerprint4.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestSqrDistances.get(0) >= 0.0);

            avgValidSignalDistance += nearestSqrDistances.get(0);
            avgValidDistance += closestFingerprint1.getPosition().distanceTo(position);
            numValid++;

            //force IllegalArgumentException
            try {
                WifiKNearestFinder.findNearestTo(null, fingerprints);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            try {
                WifiKNearestFinder.findNearestTo(fingerprint, null);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }

        assertTrue(numValid > 0);

        avgSignalDistance /= TIMES;
        avgDistance /= TIMES;
        avgValidSignalDistance /= numValid;
        avgValidDistance /= numValid;

        LOGGER.log(Level.INFO, "Average signal distance: {0}", avgSignalDistance);
        LOGGER.log(Level.INFO, "Average position distance: {0}", avgDistance);
        LOGGER.log(Level.INFO, "Average valid signal distance: {0}", avgValidSignalDistance);
        LOGGER.log(Level.INFO, "Average valid position distance: {0}", avgValidDistance);
    }

    @Test
    public void testFindKNearestTo() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find k closest fingerprints
            int k = randomizer.nextInt(2, numFingerprints);
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> kClosestFingerprints1 =
                    finder.findKNearestTo(fingerprint, k);
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> kClosestFingerprints2 =
                    WifiKNearestFinder.findKNearestTo(fingerprint, fingerprints, k);

            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> kClosestFingerprints3 =
                    new ArrayList<>();
            List<Double> nearestSqrDistances3 = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k, kClosestFingerprints3,
                    nearestSqrDistances3);

            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> kClosestFingerprints4 =
                    new ArrayList<>();
            List<Double> nearestSqrDistances4 = new ArrayList<>();
            WifiKNearestFinder.findKNearestTo(fingerprint, fingerprints, k,
                    kClosestFingerprints4, nearestSqrDistances4);

            //check
            assertEquals(kClosestFingerprints1.size(), k);
            assertEquals(kClosestFingerprints2.size(), k);
            assertEquals(kClosestFingerprints3.size(), k);
            assertEquals(kClosestFingerprints4.size(), k);
            assertEquals(nearestSqrDistances3.size(), k);
            assertEquals(nearestSqrDistances4.size(), k);

            for(int i = 1; i < k; i++) {
                assertTrue(nearestSqrDistances3.get(i -1) <=
                        nearestSqrDistances3.get(i));
                assertTrue(nearestSqrDistances4.get(i -1) <=
                        nearestSqrDistances4.get(i));
            }

            Point2D nearestPosition = tree.nearestPoint(position);

            //check that k nearest fingerprints contains closest one
            boolean found = false;
            for (int i = 0; i < k; i++) {
                RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> fingerprint1 =
                        kClosestFingerprints1.get(i);
                RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> fingerprint2 =
                        kClosestFingerprints2.get(i);
                RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> fingerprint3 =
                        kClosestFingerprints3.get(i);
                RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> fingerprint4 =
                        kClosestFingerprints4.get(i);

                assertEquals(fingerprint1.getPosition(), fingerprint2.getPosition());
                assertEquals(fingerprint2.getPosition(), fingerprint3.getPosition());
                assertEquals(fingerprint3.getPosition(), fingerprint4.getPosition());
                assertEquals(nearestSqrDistances3.get(i), nearestSqrDistances4.get(i));

                if(fingerprint1.getPosition().equals(nearestPosition, ABSOLUTE_ERROR)) {
                    found = true;
                    break;
                }
            }

            if(found) {
                numValid++;
            }

            //force IllegalArgumentException
            try {
                WifiKNearestFinder.findKNearestTo(null, fingerprints, k,
                        kClosestFingerprints4, nearestSqrDistances4);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            try {
                WifiKNearestFinder.findKNearestTo(fingerprint, null, k,
                        kClosestFingerprints4, nearestSqrDistances4);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            try {
                WifiKNearestFinder.findKNearestTo(fingerprint, fingerprints, k,
                        null, nearestSqrDistances4);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            try {
                WifiKNearestFinder.findKNearestTo(fingerprint, fingerprints, k,
                        kClosestFingerprints4, null);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testFindKNearestToAll() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints =
                    new ArrayList<>();
            List<Double> nearestSqrDistances = new ArrayList<>();
            WifiKNearestFinder.findKNearestTo(fingerprint, fingerprints,
                    numFingerprints, closestFingerprints, nearestSqrDistances);

            //check
            for (int i = 1; i < numFingerprints; i++) {
                assertTrue(nearestSqrDistances.get(i - 1) <=
                        nearestSqrDistances.get(i));
            }

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testFindNearestToWithError() {
        int numValid = 0;
        double avgValidSignalDistance = 0.0;
        double avgValidDistance = 0.0;
        double avgSignalDistance = 0.0;
        double avgDistance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double error = errorRandomizer.nextDouble();
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find closest fingerprint
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint1 =
                    finder.findNearestTo(fingerprint);
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint2 =
                    WifiKNearestFinder.findNearestTo(fingerprint, fingerprints);

            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints1 =
                    finder.findKNearestTo(fingerprint, 1);
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints2 =
                    new ArrayList<>();
            List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints2,
                    nearestSqrDistances);

            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint3 =
                    closestFingerprints1.get(0);
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint4 =
                    closestFingerprints2.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint1.getPosition().distanceTo(position);

            Point2D nearestPosition = tree.nearestPoint(position);
            if (!nearestPosition.equals(
                    closestFingerprint1.getPosition(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(nearestPosition.equals(
                    closestFingerprint1.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(
                    closestFingerprint2.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(
                    closestFingerprint3.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestPosition.equals(
                    closestFingerprint4.getPosition(), ABSOLUTE_ERROR));
            assertTrue(nearestSqrDistances.get(0) >= 0.0);

            avgValidSignalDistance += nearestSqrDistances.get(0);
            avgValidDistance += closestFingerprint1.getPosition().distanceTo(position);
            numValid++;
        }

        assertTrue(numValid > 0);

        avgSignalDistance /= TIMES;
        avgDistance /= TIMES;
        avgValidSignalDistance /= numValid;
        avgValidDistance /= numValid;

        LOGGER.log(Level.INFO, "Average signal distance: {0}", avgSignalDistance);
        LOGGER.log(Level.INFO, "Average position distance: {0}", avgDistance);
        LOGGER.log(Level.INFO, "Average valid signal distance: {0}", avgValidSignalDistance);
        LOGGER.log(Level.INFO, "Average valid position distance: {0}", avgValidDistance);
    }

    @Test
    public void testFindBestK() {
        double avgK = 0.0;
        for(int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            Point2D nearestPosition = tree.nearestPoint(position);

            for (int k = 1; k < numFingerprints; k++) {
                List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprints =
                        finder.findKNearestTo(fingerprint, k);
                boolean found = false;
                for (int i = 0; i < k; i++) {
                    if (nearestFingerprints.get(i).getPosition().equals(
                            nearestPosition, ABSOLUTE_ERROR)) {
                        avgK += k;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }
        }

        avgK /= TIMES;
        LOGGER.log(Level.INFO, "Average best k: {0}", avgK);
    }

    @Test
    public void testFindBestKWithError() {
        double avgK = 0.0;
        for(int t = 0; t < TIMES; t++) {
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double error = errorRandomizer.nextDouble();
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            Point2D nearestPosition = tree.nearestPoint(position);

            for (int k = 1; k < numFingerprints; k++) {
                List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprints =
                        finder.findKNearestTo(fingerprint, k);
                boolean found = false;
                for (int i = 0; i < k; i++) {
                    if (nearestFingerprints.get(i).getPosition().equals(
                            nearestPosition, ABSOLUTE_ERROR)) {
                        avgK += k;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }
        }

        avgK /= TIMES;
        LOGGER.log(Level.INFO, "Average best k: {0}", avgK);
    }

    @Test
    public void testFindNearestToUniformFingerprints() {
        double avgSignalDistance = 0.0;
        double avgDistance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            //setup uniform fingerprint readings
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x+= SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y+= SEPARATION_POS) {
                    InhomogeneousPoint2D fingerprintPosition =
                            new InhomogeneousPoint2D(x, y);

                    List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                    for (int j = 0; j < numAccessPoints; j++) {
                        double distance = fingerprintPosition.distanceTo(
                                accessPointPositions[j]);
                        double rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency()));
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find closest fingerprint
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints =
                    new ArrayList<>();
            List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints,
                    nearestSqrDistances);

            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint =
                    closestFingerprints.get(0);


            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint.getPosition().distanceTo(position);
        }

        avgSignalDistance /= TIMES;
        avgDistance /= TIMES;
        LOGGER.log(Level.INFO, "Average signal distance: {0}", avgSignalDistance);
        LOGGER.log(Level.INFO, "Average position distance: {0}", avgDistance);
    }

    @Test
    public void testFindNearestToWithErrorUniformFingerprints() {
        double avgSignalDistance = 0.0;
        double avgDistance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            //setup uniform fingerprint readings
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x+= SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y+= SEPARATION_POS) {
                    InhomogeneousPoint2D fingerprintPosition =
                            new InhomogeneousPoint2D(x, y);

                    List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                    for (int j = 0; j < numAccessPoints; j++) {
                        double distance = fingerprintPosition.distanceTo(
                                accessPointPositions[j]);
                        double error = errorRandomizer.nextDouble();
                        double rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find closest fingerprint
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints =
                    new ArrayList<>();
            List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints,
                    nearestSqrDistances);

            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint =
                    closestFingerprints.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint.getPosition().distanceTo(position);
        }

        avgSignalDistance /= TIMES;
        avgDistance /= TIMES;
        LOGGER.log(Level.INFO, "Average signal distance: {0}", avgSignalDistance);
        LOGGER.log(Level.INFO, "Average position distance: {0}", avgDistance);
    }

    @Test
    public void testFindBestKUniformFingerprints() {
        double avgK = 0.0;
        for(int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            //setup uniform fingerprint readings
            List<Point2D> fingerprintsPositionsList = new ArrayList<>();
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x+= SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y+= SEPARATION_POS) {
                    InhomogeneousPoint2D fingerprintPosition =
                            new InhomogeneousPoint2D(x, y);
                    fingerprintsPositionsList.add(fingerprintPosition);

                    List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                    for (int j = 0; j < numAccessPoints; j++) {
                        double distance = fingerprintPosition.distanceTo(
                                accessPointPositions[j]);
                        double rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency()));
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(fingerprintsPositionsList);

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            Point2D nearestPosition = tree.nearestPoint(position);

            int numFingerprints = fingerprints.size();
            int maxK = Math.min(numFingerprints, MAX_K);
            for (int k = 1; k < maxK; k++) {
                List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprints =
                        finder.findKNearestTo(fingerprint, k);
                boolean found = false;
                for (int i = 0; i < k; i++) {
                    if (nearestFingerprints.get(i).getPosition().equals(
                            nearestPosition, ABSOLUTE_ERROR)) {
                        avgK += k;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }
        }

        avgK /= TIMES;
        LOGGER.log(Level.INFO, "Average best k: {0}", avgK);
    }

    @Test
    public void testFindBestKWithErrorUniformFingerprints() {
        double avgK = 0.0;
        for(int t = 0; t < TIMES; t++) {
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            double[] transmittedPower = new double[numAccessPoints];
            WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            //setup uniform fingerprint readings
            List<Point2D> fingerprintsPositionsList = new ArrayList<>();
            List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x+= SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y+= SEPARATION_POS) {
                    InhomogeneousPoint2D fingerprintPosition =
                            new InhomogeneousPoint2D(x, y);
                    fingerprintsPositionsList.add(fingerprintPosition);

                    List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                    for (int j = 0; j < numAccessPoints; j++) {
                        double distance = fingerprintPosition.distanceTo(
                                accessPointPositions[j]);
                        double error = errorRandomizer.nextDouble();
                        double rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(fingerprintsPositionsList);

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            Point2D nearestPosition = tree.nearestPoint(position);

            int numFingerprints = fingerprints.size();
            int maxK = Math.min(numFingerprints, MAX_K);
            for (int k = 1; k < maxK; k++) {
                List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprints =
                        finder.findKNearestTo(fingerprint, k);
                boolean found = false;
                for (int i = 0; i < k; i++) {
                    if (nearestFingerprints.get(i).getPosition().equals(
                            nearestPosition, ABSOLUTE_ERROR)) {
                        avgK += k;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }
        }

        avgK /= TIMES;
        LOGGER.log(Level.INFO, "Average best k: {0}", avgK);
    }

    private double receivedPower(double equivalentTransmittedPower, double distance, double frequency) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), 2.0);
        return equivalentTransmittedPower * k / (distance * distance);
    }
}
