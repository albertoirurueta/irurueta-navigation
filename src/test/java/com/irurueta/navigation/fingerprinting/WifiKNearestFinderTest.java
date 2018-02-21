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
package com.irurueta.navigation.fingerprinting;

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

    private static final double ERROR_STD = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final double SPEED_OF_LIGHT = 3e8; //(m/s)

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
        List<WifiFingerprintLocated2D> fingerprints = new ArrayList<>();
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
            List<WifiFingerprintLocated2D> fingerprints = new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<WifiReading> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new WifiReading(
                            accessPoints[j], rssi));
                }

                fingerprints.add(new WifiFingerprintLocated2D(
                        readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<WifiReading> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new WifiReading(accessPoints[i], rssi));
            }
            WifiFingerprint fingerprint = new WifiFingerprint(readings);

            //find closest fingerprint
            WifiFingerprintLocated2D closestFingerprint1 =
                    (WifiFingerprintLocated2D) finder.findNearestTo(fingerprint);
            WifiFingerprintLocated2D closestFingerprint2 =
                    (WifiFingerprintLocated2D) WifiKNearestFinder.findNearestTo(fingerprint,
                            fingerprints);

            List<WifiFingerprintLocated<Point2D>> closestFingerprints1 =
                    finder.findKNearestTo(fingerprint, 1);
            List<WifiFingerprintLocated<Point2D>> closestFingerprints2 =
                    new ArrayList<>();
            List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints2,
                    nearestSqrDistances);

            WifiFingerprintLocated2D closestFingerprint3 =
                    (WifiFingerprintLocated2D) closestFingerprints1.get(0);
            WifiFingerprintLocated2D closestFingerprint4 =
                    (WifiFingerprintLocated2D) closestFingerprints2.get(0);

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

            avgDistance += nearestSqrDistances.get(0);
            numValid++;
        }

        assertTrue(numValid > 0);

        avgDistance /= numValid;
        LOGGER.log(Level.INFO, "Average distance: {0}", avgDistance);
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
            List<WifiFingerprintLocated2D> fingerprints = new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<WifiReading> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new WifiReading(
                            accessPoints[j], rssi));
                }

                fingerprints.add(new WifiFingerprintLocated2D(
                        readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<WifiReading> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new WifiReading(accessPoints[i], rssi));
            }
            WifiFingerprint fingerprint = new WifiFingerprint(readings);

            //find k closest fingerprints
            int k = randomizer.nextInt(2, numFingerprints);
            List<WifiFingerprintLocated<Point2D>> kClosestFingerprints1 =
                    finder.findKNearestTo(fingerprint, k);
            List<WifiFingerprintLocated<Point2D>> kClosestFingerprints2 =
                    WifiKNearestFinder.findKNearestTo(fingerprint, fingerprints, k);

            List<WifiFingerprintLocated<Point2D>> kClosestFingerprints3 =
                    new ArrayList<>();
            List<Double> nearestSqrDistances3 = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k, kClosestFingerprints3,
                    nearestSqrDistances3);

            List<WifiFingerprintLocated<Point2D>> kClosestFingerprints4 =
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
                WifiFingerprintLocated2D fingerprint1 =
                        (WifiFingerprintLocated2D)kClosestFingerprints1.get(i);
                WifiFingerprintLocated2D fingerprint2 =
                        (WifiFingerprintLocated2D)kClosestFingerprints2.get(i);
                WifiFingerprintLocated2D fingerprint3 =
                        (WifiFingerprintLocated2D)kClosestFingerprints3.get(i);
                WifiFingerprintLocated2D fingerprint4 =
                        (WifiFingerprintLocated2D)kClosestFingerprints4.get(i);

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
            List<WifiFingerprintLocated2D> fingerprints = new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<WifiReading> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new WifiReading(
                            accessPoints[j], rssi));
                }

                fingerprints.add(new WifiFingerprintLocated2D(
                        readings, fingerprintsPositions[i]));
            }

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<WifiReading> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new WifiReading(accessPoints[i], rssi));
            }
            WifiFingerprint fingerprint = new WifiFingerprint(readings);

            List<WifiFingerprintLocated<Point2D>> closestFingerprints =
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
            List<WifiFingerprintLocated2D> fingerprints = new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<WifiReading> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double error = errorRandomizer.nextDouble();
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new WifiReading(
                            accessPoints[j], rssi));
                }

                fingerprints.add(new WifiFingerprintLocated2D(
                        readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<WifiReading> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new WifiReading(accessPoints[i], rssi));
            }
            WifiFingerprint fingerprint = new WifiFingerprint(readings);

            //find closest fingerprint
            WifiFingerprintLocated2D closestFingerprint1 =
                    (WifiFingerprintLocated2D) finder.findNearestTo(fingerprint);
            WifiFingerprintLocated2D closestFingerprint2 =
                    (WifiFingerprintLocated2D) WifiKNearestFinder.findNearestTo(fingerprint,
                            fingerprints);

            List<WifiFingerprintLocated<Point2D>> closestFingerprints1 =
                    finder.findKNearestTo(fingerprint, 1);
            List<WifiFingerprintLocated<Point2D>> closestFingerprints2 =
                    new ArrayList<>();
            List<Double> nearestSqrDistances = new ArrayList<>();
            finder.findKNearestTo(fingerprint, 1, closestFingerprints2,
                    nearestSqrDistances);

            WifiFingerprintLocated2D closestFingerprint3 =
                    (WifiFingerprintLocated2D) closestFingerprints1.get(0);
            WifiFingerprintLocated2D closestFingerprint4 =
                    (WifiFingerprintLocated2D) closestFingerprints2.get(0);

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

            avgDistance += nearestSqrDistances.get(0);
            numValid++;
        }

        assertTrue(numValid > 0);

        avgDistance /= numValid;
        LOGGER.log(Level.INFO, "Average distance: {0}", avgDistance);
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
            List<WifiFingerprintLocated2D> fingerprints = new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<WifiReading> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new WifiReading(
                            accessPoints[j], rssi));
                }

                fingerprints.add(new WifiFingerprintLocated2D(
                        readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<WifiReading> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new WifiReading(accessPoints[i], rssi));
            }
            WifiFingerprint fingerprint = new WifiFingerprint(readings);

            Point2D nearestPosition = tree.nearestPoint(position);

            for (int k = 1; k < numFingerprints; k++) {
                List<WifiFingerprintLocated<Point2D>> nearestFingerprints =
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
            List<WifiFingerprintLocated2D> fingerprints = new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<WifiReading> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    double error = errorRandomizer.nextDouble();
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new WifiReading(
                            accessPoints[j], rssi));
                }

                fingerprints.add(new WifiFingerprintLocated2D(
                        readings, fingerprintsPositions[i]));
            }

            WifiKNearestFinder<Point2D> finder =
                    new WifiKNearestFinder<>(fingerprints);

            //build tree of fingerprint positions
            KDTree2D tree = new KDTree2D(Arrays.asList(fingerprintsPositions));

            //generate measurement at random position
            Point2D position = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            List<WifiReading> readings = new ArrayList<>();
            for (int i = 0; i < numAccessPoints; i++) {
                double distance = position.distanceTo(accessPointPositions[i]);
                double error = errorRandomizer.nextDouble();
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency())) + error;
                readings.add(new WifiReading(accessPoints[i], rssi));
            }
            WifiFingerprint fingerprint = new WifiFingerprint(readings);

            Point2D nearestPosition = tree.nearestPoint(position);

            for (int k = 1; k < numFingerprints; k++) {
                List<WifiFingerprintLocated<Point2D>> nearestFingerprints =
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

    private double dBmToPower(double dBm) {
        return Math.pow(10.0, dBm / 10.0);
    }

    private double powerTodBm(double mW) {
        return 10.0 * Math.log10(mW);
    }
}
