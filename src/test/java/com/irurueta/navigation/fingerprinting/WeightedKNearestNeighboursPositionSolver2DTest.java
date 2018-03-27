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
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
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
import static com.irurueta.navigation.fingerprinting.Utils.*;

public class WeightedKNearestNeighboursPositionSolver2DTest implements
        WeightedKNearestNeighboursPositionSolverListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            WeightedKNearestNeighboursPositionSolver2DTest.class.getName());

    private static final int MIN_RSSI = -100;
    private static final int MAX_RSSI = -50;

    private static final int MIN_AP = 1;
    private static final int MAX_AP = 5;

    private static final int MIN_FINGERPRINTS = 50;
    private static final int MAX_FINGERPRINTS = 100;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double SEPARATION_POS = 2.0;

    private static final double ERROR_STD = 0.5;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 1000;
    private static final int SHORT_TIMES = 25;

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final double SPEED_OF_LIGHT = 3e8; //(m/s)

    private static final int MAX_K = 10;

    private int solveStart;
    private int solveEnd;

    public WeightedKNearestNeighboursPositionSolver2DTest() { }

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
        //test empty constructor
        WeightedKNearestNeighboursPositionSolver2D solver =
                new WeightedKNearestNeighboursPositionSolver2D();

        //check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNull(solver.getListener());
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());


        //test constructor with fingerprints and distances
        //noinspection all
        RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] fingerprints =
                new RssiFingerprintLocated[1];
        double[] distances = new double[1];
        solver = new WeightedKNearestNeighboursPositionSolver2D(fingerprints, distances);

        //check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertNull(solver.getListener());
        assertSame(solver.getFingerprints(), fingerprints);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new WeightedKNearestNeighboursPositionSolver2D(
                    null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new WeightedKNearestNeighboursPositionSolver2D(
                    fingerprints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //noinspection all
            solver = new WeightedKNearestNeighboursPositionSolver2D(
                    new RssiFingerprintLocated[0], new double[0]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new WeightedKNearestNeighboursPositionSolver2D(
                    fingerprints, new double[2]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);


        //test constructor with listener
        solver = new WeightedKNearestNeighboursPositionSolver2D(this);

        //check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertSame(solver.getListener(), this);
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());


        //test constructor with fingerprints, distances and listener
        solver = new WeightedKNearestNeighboursPositionSolver2D(
                fingerprints, distances, this);

        //check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 2);
        assertSame(solver.getListener(), this);
        assertSame(solver.getFingerprints(), fingerprints);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());

        //Force IllegalArgumentException
        solver = null;
        try {
            solver = new WeightedKNearestNeighboursPositionSolver2D(
                    null, distances, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new WeightedKNearestNeighboursPositionSolver2D(
                    fingerprints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //noinspection all
            solver = new WeightedKNearestNeighboursPositionSolver2D(
                    new RssiFingerprintLocated[0], new double[0], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver = new WeightedKNearestNeighboursPositionSolver2D(
                    fingerprints, new double[2], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(solver);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        WeightedKNearestNeighboursPositionSolver2D solver =
                new WeightedKNearestNeighboursPositionSolver2D();

        //check default value
        assertNull(solver.getListener());

        //set new value
        solver.setListener(this);

        //check
        assertSame(solver.getListener(), this);
    }

    @Test
    public void testSetFingerprintsAndDistances() throws LockedException {
        WeightedKNearestNeighboursPositionSolver2D solver =
                new WeightedKNearestNeighboursPositionSolver2D();

        //check default values
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());

        //set new values
        //noinspection all
        RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] fingerprints =
                new RssiFingerprintLocated[1];
        double[] distances = new double[1];
        solver.setFingerprintsAndDistances(fingerprints, distances);

        //check
        assertSame(solver.getFingerprints(), fingerprints);
        assertSame(solver.getDistances(), distances);

        //force IllegalArgumentException
        try {
            solver.setFingerprintsAndDistances(null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setFingerprintsAndDistances(fingerprints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            //noinspection all
            solver.setFingerprintsAndDistances(
                    new RssiFingerprintLocated[0], new double[0]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            solver.setFingerprintsAndDistances(fingerprints, new double[2]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetEpsilon() throws LockedException {
        WeightedKNearestNeighboursPositionSolver2D solver =
                new WeightedKNearestNeighboursPositionSolver2D();

        //check default value
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);

        //set new value
        solver.setEpsilon(1.0);

        //check
        assertEquals(solver.getEpsilon(), 1.0, 0.0);

        //force IllegalArgumentException
        try {
            solver.setEpsilon(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testSolve1Fingerprint() throws NotReadyException, LockedException {
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
                    readings.add(new RssiReading<>(
                            accessPoints[j], rssi));
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

            //find nearest fingerprint
            int k = 1;
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprintsList =
                    new ArrayList<>();
            List<Double> nearestDistancesList = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k,
                    nearestFingerprintsList, nearestDistancesList);

            //noinspection all
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            double[] nearestDistances = new double[k];
            for(int i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            WeightedKNearestNeighboursPositionSolver2D solver =
                    new WeightedKNearestNeighboursPositionSolver2D(
                            nearestFingerprints, nearestDistances, this);

            //solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);

            solver.solve();

            //check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);

            Point2D nearestPosition = tree.nearestPoint(position);
            Point2D estimatedPosition = solver.getEstimatedPosition();

            //estimated position is always equal to provided fingerprint when
            //only one is provided
            assertEquals(estimatedPosition, nearestFingerprints[0].getPosition());

            if (nearestPosition.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                numValid++;
            }
        }

        assertTrue(numValid > 0);

        //force NotReadyException
        WeightedKNearestNeighboursPositionSolver2D solver =
                new WeightedKNearestNeighboursPositionSolver2D();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testSolveKFingerprints() throws NotReadyException, LockedException {
        for (int k = 2; k < MAX_K; k++) {
            solveKFingerprints(k, 0.0);
        }
    }

    @Test
    public void testSolveKFingerprintsWithError() throws NotReadyException, LockedException {
        for (int k = 2; k < MAX_K; k++) {
            solveKFingerprints(k, ERROR_STD);
        }
    }

    @Test
    public void testFindBestK() throws NotReadyException, LockedException {
        findBestK(0.0);
    }

    @Test
    public void testFindBestKWithError() throws NotReadyException, LockedException {
        findBestK(ERROR_STD);
    }

    @Test
    public void testSolveKFingerprintsUniformFingerprints() throws NotReadyException, LockedException {
        for (int k = 2; k < MAX_K; k++) {
            solveKFingerprintsUniformFingerprints(k, 0.0);
        }
    }

    @Test
    public void testSolveKFingerprintsWithErrorUniformFingerprints() throws NotReadyException, LockedException {
        for (int k = 2; k < MAX_K; k++) {
            solveKFingerprintsUniformFingerprints(k, ERROR_STD);
        }
    }

    @Test
    public void testFindBestKUniformFingerprints() throws NotReadyException, LockedException {
        findBestKUniformFingerprints(0.0);
    }

    @Test
    public void testFindBestKWithErrorUniformFingerprints() throws NotReadyException, LockedException {
        findBestKUniformFingerprints(ERROR_STD);
    }

    @Override
    public void onSolveStart(WeightedKNearestNeighboursPositionSolver<Point2D> solver) {
        solveStart++;
        checkLocked((WeightedKNearestNeighboursPositionSolver2D)solver);
    }

    @Override
    public void onSolveEnd(WeightedKNearestNeighboursPositionSolver<Point2D> solver) {
        solveEnd++;
        checkLocked((WeightedKNearestNeighboursPositionSolver2D)solver);
    }

    private void solveKFingerprints(int k, double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(new Random(), 0.0, errorStd);
        }

        int numValid = 0;
        double avgDistance = 0.0;
        double avgImprovedDistance = 0.0;
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
                    double error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
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
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find nearest fingerprints
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprintsList =
                    new ArrayList<>();
            List<Double> nearestDistancesList = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k,
                    nearestFingerprintsList, nearestDistancesList);

            //noinspection all
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            double[] nearestDistances = new double[k];
            for(int i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            WeightedKNearestNeighboursPositionSolver2D solver =
                    new WeightedKNearestNeighboursPositionSolver2D(
                            nearestFingerprints, nearestDistances, this);

            //solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);

            solver.solve();

            //check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);

            Point2D nearestPosition = tree.nearestPoint(position);
            Point2D estimatedPosition = solver.getEstimatedPosition();

            //check if estimated position is closer to the actual position than
            //nearest fingerprint
            double distance = estimatedPosition.distanceTo(position);
            avgDistance += distance;

            if(distance <= nearestPosition.distanceTo(position)) {
                avgImprovedDistance += distance;
                numValid++;
            }
        }

        assertTrue(numValid > 0);

        avgDistance /= TIMES;
        avgImprovedDistance /= numValid;

        LOGGER.log(Level.INFO, "{0} of {1} estimated positions have improved with {2} neighbours",
                new Object[]{numValid, TIMES, k});
        LOGGER.log(Level.INFO, "Average error distance: {0} meters, Average improved error distance: {1} meters",
                new Object[]{avgDistance, avgImprovedDistance});
    }

    private void findBestK(double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(new Random(), 0.0, errorStd);
        }

        double avgBestK = 0.0;
        double avgBestDistance = 0.0;
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
                    double error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
                    double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated2D<>(readings, fingerprintsPositions[i]));
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

            int bestK = 0;
            double bestDistance = Double.MAX_VALUE;
            int maxK = Math.min(numFingerprints, MAX_K);
            for (int k = 1; k < maxK; k++) {
                List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprintsList =
                        new ArrayList<>();
                List<Double> nearestDistancesList = new ArrayList<>();
                finder.findKNearestTo(fingerprint, k,
                        nearestFingerprintsList, nearestDistancesList);

                //noinspection all
                RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] nearestFingerprints =
                        new RssiFingerprintLocated[k];
                double[] nearestDistances = new double[k];
                for (int i = 0; i < k; i++) {
                    nearestFingerprints[i] = nearestFingerprintsList.get(i);
                    nearestDistances[i] = nearestDistancesList.get(i);
                }

                WeightedKNearestNeighboursPositionSolver2D solver =
                        new WeightedKNearestNeighboursPositionSolver2D(
                                nearestFingerprints, nearestDistances);

                solver.solve();

                Point2D estimatedPosition = solver.getEstimatedPosition();

                double distance = estimatedPosition.distanceTo(position);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestK = k;
                }
            }

            avgBestK += bestK;
            avgBestDistance += bestDistance;
        }

        avgBestK /= TIMES;
        avgBestDistance /= TIMES;
        LOGGER.log(Level.INFO, "Best number of neighbours: {0}, Average error distance: {1} meters",
                new Object[]{avgBestK, avgBestDistance});
    }

    private void solveKFingerprintsUniformFingerprints(int k, double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(new Random(), 0.0, errorStd);
        }

        int numValid = 0;
        double avgDistance = 0.0;
        double avgImprovedDistance = 0.0;
        for (int t = 0; t < SHORT_TIMES; t++) {
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
                        double error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
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
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find nearest fingerprints
            List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprintsList =
                    new ArrayList<>();
            List<Double> nearestDistancesList = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k,
                    nearestFingerprintsList, nearestDistancesList);

            //noinspection all
            RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            double[] nearestDistances = new double[k];
            for(int i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            WeightedKNearestNeighboursPositionSolver2D solver =
                    new WeightedKNearestNeighboursPositionSolver2D(
                            nearestFingerprints, nearestDistances, this);

            //solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);

            solver.solve();

            //check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);

            Point2D nearestPosition = tree.nearestPoint(position);
            Point2D estimatedPosition = solver.getEstimatedPosition();

            //check if estimated position is closer to the actual position than
            //nearest fingerprint
            double distance = estimatedPosition.distanceTo(position);
            avgDistance += distance;

            if(distance <= nearestPosition.distanceTo(position)) {
                avgImprovedDistance += distance;
                numValid++;
            }
        }

        if (numValid > 0) {
            avgImprovedDistance /= numValid;
            avgDistance /= SHORT_TIMES;
            LOGGER.log(Level.INFO, "{0} of {1} estimated positions have improved with {2} neighbours",
                    new Object[]{numValid, SHORT_TIMES, k});
            LOGGER.log(Level.INFO, "Average error distance: {0} meters, Average improved error distance: {1} meters",
                    new Object[]{avgDistance, avgImprovedDistance});
        }
    }

    private void findBestKUniformFingerprints(double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(new Random(), 0.0, errorStd);
        }

        double avgBestK = 0.0;
        double avgBestDistance = 0.0;
        for (int t = 0; t < SHORT_TIMES; t++) {
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
                        double error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
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
                double rssi = powerTodBm(receivedPower(
                        transmittedPower[i], distance, accessPoints[i].getFrequency()));
                readings.add(new RssiReading<>(accessPoints[i], rssi));
            }
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                    new RssiFingerprint<>(readings);

            int bestK = 0;
            double bestDistance = Double.MAX_VALUE;
            int numFingerprints = fingerprints.size();
            int maxK = Math.min(numFingerprints, MAX_K);
            for (int k = 1; k < maxK; k++) {
                List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>> nearestFingerprintsList =
                        new ArrayList<>();
                List<Double> nearestDistancesList = new ArrayList<>();
                finder.findKNearestTo(fingerprint, k,
                        nearestFingerprintsList, nearestDistancesList);

                //noinspection all
                RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point2D>[] nearestFingerprints =
                        new RssiFingerprintLocated[k];
                double[] nearestDistances = new double[k];
                for (int i = 0; i < k; i++) {
                    nearestFingerprints[i] = nearestFingerprintsList.get(i);
                    nearestDistances[i] = nearestDistancesList.get(i);
                }

                WeightedKNearestNeighboursPositionSolver2D solver =
                        new WeightedKNearestNeighboursPositionSolver2D(
                                nearestFingerprints, nearestDistances);

                solver.solve();

                Point2D estimatedPosition = solver.getEstimatedPosition();

                double distance = estimatedPosition.distanceTo(position);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestK = k;
                }
            }

            avgBestK += bestK;
            avgBestDistance += bestDistance;
        }

        avgBestK /= SHORT_TIMES;
        avgBestDistance /= SHORT_TIMES;
        LOGGER.log(Level.INFO, "Best number of neighbours: {0}, Average error distance: {1} meters",
                new Object[]{avgBestK, avgBestDistance});
    }

    private void reset() {
        solveStart = solveEnd = 0;
    }

    private void checkLocked(WeightedKNearestNeighboursPositionSolver2D solver) {
        try {
            solver.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setFingerprintsAndDistances(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.setEpsilon(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            solver.solve();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
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
