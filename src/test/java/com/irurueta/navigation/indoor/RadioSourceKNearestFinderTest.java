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
import com.irurueta.geometry.Point3D;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static com.irurueta.navigation.indoor.Utils.dBmToPower;
import static com.irurueta.navigation.indoor.Utils.powerTodBm;
import static org.junit.Assert.*;

public class RadioSourceKNearestFinderTest {

    private static final Logger LOGGER = Logger.getLogger(
            RadioSourceKNearestFinderTest.class.getName());

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

    public RadioSourceKNearestFinderTest() {
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
    public void testConstructorWifi2D() {
        final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                new ArrayList<>();
        RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder = new RadioSourceKNearestFinder<>(
                fingerprints);

        // check
        assertSame(finder.getFingerprints(), fingerprints);

        // Force IllegalArgumentException
        finder = null;
        try {
            finder = new RadioSourceKNearestFinder<>(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(finder);
    }

    @Test
    public void testConstructorWifi3D() {
        final List<RssiFingerprintLocated3D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                new ArrayList<>();
        RadioSourceKNearestFinder<Point3D, WifiAccessPoint> finder = new RadioSourceKNearestFinder<>(
                fingerprints);

        // check
        assertSame(finder.getFingerprints(), fingerprints);

        // Force IllegalArgumentException
        finder = null;
        try {
            finder = new RadioSourceKNearestFinder<>(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(finder);
    }

    @Test
    public void testConstructorBeacon2D() {
        final List<RssiFingerprintLocated2D<Beacon, RssiReading<Beacon>>> fingerprints =
                new ArrayList<>();
        RadioSourceKNearestFinder<Point2D, Beacon> finder = new RadioSourceKNearestFinder<>(
                fingerprints);

        // check
        assertSame(finder.getFingerprints(), fingerprints);

        //Force IllegalArgumentException
        finder = null;
        try {
            finder = new RadioSourceKNearestFinder<>(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(finder);
    }

    @Test
    public void testConstructorBeacon3D() {
        final List<RssiFingerprintLocated3D<Beacon, RssiReading<Beacon>>> fingerprints =
                new ArrayList<>();
        RadioSourceKNearestFinder<Point3D, Beacon> finder = new RadioSourceKNearestFinder<>(
                fingerprints);

        // check
        assertSame(finder.getFingerprints(), fingerprints);

        // Force IllegalArgumentException
        finder = null;
        try {
            finder = new RadioSourceKNearestFinder<>(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
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
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find closest fingerprint
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint1 =
                    finder.findNearestTo(fingerprint);
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint2 =
                    RadioSourceKNearestFinder.findNearestTo(fingerprint, fingerprints);

            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints1 =
                    finder.findKNearestTo(fingerprint, 1);
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints2 =
                    new ArrayList<>();
            final List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints2,
                    nearestSqrDistances);

            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint3 =
                    closestFingerprints1.get(0);
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint4 =
                    closestFingerprints2.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint1.getPosition().distanceTo(position);

            final Point2D nearestPosition = tree.nearestPoint(position);
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

            // force IllegalArgumentException
            try {
                RadioSourceKNearestFinder.findNearestTo(null, fingerprints);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                RadioSourceKNearestFinder.findNearestTo(fingerprint, null);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
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
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find k closest fingerprints
            final int k = randomizer.nextInt(2, numFingerprints);
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> kClosestFingerprints1 =
                    finder.findKNearestTo(fingerprint, k);
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> kClosestFingerprints2 =
                    RadioSourceKNearestFinder.findKNearestTo(fingerprint, fingerprints, k);

            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> kClosestFingerprints3 =
                    new ArrayList<>();
            final List<Double> nearestSqrDistances3 = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k, kClosestFingerprints3,
                    nearestSqrDistances3);

            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> kClosestFingerprints4 =
                    new ArrayList<>();
            final List<Double> nearestSqrDistances4 = new ArrayList<>();
            RadioSourceKNearestFinder.findKNearestTo(fingerprint, fingerprints, k,
                    kClosestFingerprints4, nearestSqrDistances4);

            // check
            assertEquals(kClosestFingerprints1.size(), k);
            assertEquals(kClosestFingerprints2.size(), k);
            assertEquals(kClosestFingerprints3.size(), k);
            assertEquals(kClosestFingerprints4.size(), k);
            assertEquals(nearestSqrDistances3.size(), k);
            assertEquals(nearestSqrDistances4.size(), k);

            for (int i = 1; i < k; i++) {
                assertTrue(nearestSqrDistances3.get(i - 1) <=
                        nearestSqrDistances3.get(i));
                assertTrue(nearestSqrDistances4.get(i - 1) <=
                        nearestSqrDistances4.get(i));
            }

            final Point2D nearestPosition = tree.nearestPoint(position);

            // check that k nearest fingerprints contains closest one
            boolean found = false;
            for (int i = 0; i < k; i++) {
                final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> fingerprint1 =
                        kClosestFingerprints1.get(i);
                final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> fingerprint2 =
                        kClosestFingerprints2.get(i);
                final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> fingerprint3 =
                        kClosestFingerprints3.get(i);
                final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> fingerprint4 =
                        kClosestFingerprints4.get(i);

                assertEquals(fingerprint1.getPosition(), fingerprint2.getPosition());
                assertEquals(fingerprint2.getPosition(), fingerprint3.getPosition());
                assertEquals(fingerprint3.getPosition(), fingerprint4.getPosition());
                assertEquals(nearestSqrDistances3.get(i), nearestSqrDistances4.get(i));

                if (fingerprint1.getPosition().equals(nearestPosition, ABSOLUTE_ERROR)) {
                    found = true;
                    break;
                }
            }

            if (found) {
                numValid++;
            }

            // force IllegalArgumentException
            try {
                RadioSourceKNearestFinder.findKNearestTo(null, fingerprints, k,
                        kClosestFingerprints4, nearestSqrDistances4);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                RadioSourceKNearestFinder.findKNearestTo(fingerprint, null, k,
                        kClosestFingerprints4, nearestSqrDistances4);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                RadioSourceKNearestFinder.findKNearestTo(fingerprint, fingerprints, k,
                        null, nearestSqrDistances4);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                RadioSourceKNearestFinder.findKNearestTo(fingerprint, fingerprints, k,
                        kClosestFingerprints4, null);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testFindKNearestToAll() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints =
                    new ArrayList<>();
            final List<Double> nearestSqrDistances = new ArrayList<>();
            RadioSourceKNearestFinder.findKNearestTo(fingerprint, fingerprints,
                    numFingerprints, closestFingerprints, nearestSqrDistances);

            // check
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
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double error = errorRandomizer.nextDouble();
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double error = errorRandomizer.nextDouble();
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find closest fingerprint
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint1 =
                    finder.findNearestTo(fingerprint);
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint2 =
                    RadioSourceKNearestFinder.findNearestTo(fingerprint, fingerprints);

            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints1 =
                    finder.findKNearestTo(fingerprint, 1);
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints2 =
                    new ArrayList<>();
            final List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints2,
                    nearestSqrDistances);

            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint3 =
                    closestFingerprints1.get(0);
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint4 =
                    closestFingerprints2.get(0);

            avgSignalDistance += nearestSqrDistances.get(0);
            avgDistance += closestFingerprint1.getPosition().distanceTo(position);

            final Point2D nearestPosition = tree.nearestPoint(position);
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
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final Point2D nearestPosition = tree.nearestPoint(position);

            for (int k = 1; k < numFingerprints; k++) {
                final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprints =
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
        for (int t = 0; t < TIMES; t++) {
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point2D[] fingerprintsPositions = new Point2D[numFingerprints];
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double error = errorRandomizer.nextDouble();
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double error = errorRandomizer.nextDouble();
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final Point2D nearestPosition = tree.nearestPoint(position);

            for (int k = 1; k < numFingerprints; k++) {
                final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprints =
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
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    final InhomogeneousPoint2D fingerprintPosition =
                            new InhomogeneousPoint2D(x, y);

                    final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                    for (int j = 0; j < numAccessPoints; j++) {
                        final double distance = fingerprintPosition.distanceTo(
                                accessPointPositions[j]);
                        final double rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency()));
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find closest fingerprint
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints =
                    new ArrayList<>();
            final List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints,
                    nearestSqrDistances);

            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint =
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
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    final InhomogeneousPoint2D fingerprintPosition =
                            new InhomogeneousPoint2D(x, y);

                    final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                    for (int j = 0; j < numAccessPoints; j++) {
                        final double distance = fingerprintPosition.distanceTo(
                                accessPointPositions[j]);
                        final double error = errorRandomizer.nextDouble();
                        final double rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double error = errorRandomizer.nextDouble();
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            // find closest fingerprint
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> closestFingerprints =
                    new ArrayList<>();
            final List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints,
                    nearestSqrDistances);

            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D> closestFingerprint =
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
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final List<Point2D> fingerprintsPositionsList = new ArrayList<>();
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    final InhomogeneousPoint2D fingerprintPosition =
                            new InhomogeneousPoint2D(x, y);
                    fingerprintsPositionsList.add(fingerprintPosition);

                    final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                    for (int j = 0; j < numAccessPoints; j++) {
                        final double distance = fingerprintPosition.distanceTo(
                                accessPointPositions[j]);
                        final double rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency()));
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree2D tree = new KDTree2D(fingerprintsPositionsList);

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final Point2D nearestPosition = tree.nearestPoint(position);

            final int numFingerprints = fingerprints.size();
            final int maxK = Math.min(numFingerprints, MAX_K);
            for (int k = 1; k < maxK; k++) {
                final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprints =
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
        for (int t = 0; t < TIMES; t++) {
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point2D[] accessPointPositions = new Point2D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final List<Point2D> fingerprintsPositionsList = new ArrayList<>();
            final List<RssiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    final InhomogeneousPoint2D fingerprintPosition =
                            new InhomogeneousPoint2D(x, y);
                    fingerprintsPositionsList.add(fingerprintPosition);

                    final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                    for (int j = 0; j < numAccessPoints; j++) {
                        final double distance = fingerprintPosition.distanceTo(
                                accessPointPositions[j]);
                        final double error = errorRandomizer.nextDouble();
                        final double rssi = powerTodBm(receivedPower(
                                transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                        readings.add(new RssiReading<>(accessPoints[j], rssi));
                    }

                    fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintPosition));
                }
            }

            final RadioSourceKNearestFinder<Point2D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree2D tree = new KDTree2D(fingerprintsPositionsList);

            // generate measurement at random position
            final Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                final double distance = position.distanceTo(accessPointPositions[i]);
                final double error = errorRandomizer.nextDouble();
                final double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            final Point2D nearestPosition = tree.nearestPoint(position);

            final int numFingerprints = fingerprints.size();
            final int maxK = Math.min(numFingerprints, MAX_K);
            for (int k = 1; k < maxK; k++) {
                final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprints =
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

    private double receivedPower(
            final double equivalentTransmittedPower, final double distance, final double frequency) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * frequency), 2.0);
        return equivalentTransmittedPower * k / (distance * distance);
    }
}
