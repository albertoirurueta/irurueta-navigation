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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.KDTree3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
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

public class WeightedKNearestNeighboursPositionSolver3DTest implements
        WeightedKNearestNeighboursPositionSolverListener<Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            WeightedKNearestNeighboursPositionSolver3DTest.class.getName());

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

    private static final int TIMES = 100;
    private static final int SHORT_TIMES = 25;

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final double SPEED_OF_LIGHT = 3e8; //(m/s)

    private static final int MAX_K = 10;

    private int solveStart;
    private int solveEnd;

    public WeightedKNearestNeighboursPositionSolver3DTest() {
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
    public void testConstructor() {
        // test empty constructor
        WeightedKNearestNeighboursPositionSolver3D solver =
                new WeightedKNearestNeighboursPositionSolver3D();

        // check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNull(solver.getListener());
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());


        // test constructor with fingerprints and distances
        // noinspection unchecked
        final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] fingerprints =
                new RssiFingerprintLocated[1];
        final double[] distances = new double[1];
        solver = new WeightedKNearestNeighboursPositionSolver3D(fingerprints, distances);

        // check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertNull(solver.getListener());
        assertSame(solver.getFingerprints(), fingerprints);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new WeightedKNearestNeighboursPositionSolver3D(
                    null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new WeightedKNearestNeighboursPositionSolver3D(
                    fingerprints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // noinspection unchecked
            solver = new WeightedKNearestNeighboursPositionSolver3D(
                    new RssiFingerprintLocated[0], new double[0]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new WeightedKNearestNeighboursPositionSolver3D(
                    fingerprints, new double[2]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);


        // test constructor with listener
        solver = new WeightedKNearestNeighboursPositionSolver3D(this);

        // check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertSame(solver.getListener(), this);
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());
        assertFalse(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());


        // test constructor with fingerprints, distances and listener
        solver = new WeightedKNearestNeighboursPositionSolver3D(
                fingerprints, distances, this);

        // check default values
        assertNull(solver.getEstimatedPosition());
        assertEquals(solver.getNumberOfDimensions(), 3);
        assertSame(solver.getListener(), this);
        assertSame(solver.getFingerprints(), fingerprints);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver.isReady());
        assertFalse(solver.isLocked());
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);
        assertNull(solver.getEstimatedPositionCoordinates());

        // Force IllegalArgumentException
        solver = null;
        try {
            solver = new WeightedKNearestNeighboursPositionSolver3D(
                    null, distances, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new WeightedKNearestNeighboursPositionSolver3D(
                    fingerprints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // noinspection all
            solver = new WeightedKNearestNeighboursPositionSolver3D(
                    new RssiFingerprintLocated[0], new double[0], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver = new WeightedKNearestNeighboursPositionSolver3D(
                    fingerprints, new double[2], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(solver);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final WeightedKNearestNeighboursPositionSolver3D solver =
                new WeightedKNearestNeighboursPositionSolver3D();

        // check default value
        assertNull(solver.getListener());

        // set new value
        solver.setListener(this);

        // check
        assertSame(solver.getListener(), this);
    }

    @Test
    public void testSetFingerprintsAndDistances() throws LockedException {
        final WeightedKNearestNeighboursPositionSolver3D solver =
                new WeightedKNearestNeighboursPositionSolver3D();

        // check default values
        assertNull(solver.getFingerprints());
        assertNull(solver.getDistances());

        // set new values
        // noinspection all
        final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] fingerprints =
                new RssiFingerprintLocated[1];
        final double[] distances = new double[1];
        solver.setFingerprintsAndDistances(fingerprints, distances);

        // check
        assertSame(solver.getFingerprints(), fingerprints);
        assertSame(solver.getDistances(), distances);

        // force IllegalArgumentException
        try {
            solver.setFingerprintsAndDistances(null, distances);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setFingerprintsAndDistances(fingerprints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            // noinspection all
            solver.setFingerprintsAndDistances(
                    new RssiFingerprintLocated[0], new double[0]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            solver.setFingerprintsAndDistances(fingerprints, new double[2]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetEpsilon() throws LockedException {
        final WeightedKNearestNeighboursPositionSolver3D solver =
                new WeightedKNearestNeighboursPositionSolver3D();

        // check default value
        assertEquals(solver.getEpsilon(),
                WeightedKNearestNeighboursPositionSolver.DEFAULT_EPSILON, 0.0);

        // set new value
        solver.setEpsilon(1.0);

        // check
        assertEquals(solver.getEpsilon(), 1.0, 0.0);

        // force IllegalArgumentException
        try {
            solver.setEpsilon(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSolve1Fingerprint() throws NotReadyException, LockedException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point3D[] accessPointPositions = new Point3D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point3D[] fingerprintsPositions = new Point3D[numFingerprints];
            final List<RssiFingerprintLocated3D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency()));
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated3D<>(readings, fingerprintsPositions[i]));
            }

            final RadioSourceKNearestFinder<Point3D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree3D tree = new KDTree3D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final Point3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
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

            // find nearest fingerprint
            int k = 1;
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>> nearestFingerprintsList =
                    new ArrayList<>();
            final List<Double> nearestDistancesList = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k,
                    nearestFingerprintsList, nearestDistancesList);

            // noinspection unchecked
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            final double[] nearestDistances = new double[k];
            for (int i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            final WeightedKNearestNeighboursPositionSolver3D solver =
                    new WeightedKNearestNeighboursPositionSolver3D(
                            nearestFingerprints, nearestDistances, this);

            // solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);

            solver.solve();

            // check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);

            final Point3D nearestPosition = tree.nearestPoint(position);
            final Point3D estimatedPosition = solver.getEstimatedPosition();

            // estimated position is always equal to provided fingerprint when
            // only one is provided
            assertEquals(estimatedPosition, nearestFingerprints[0].getPosition());

            if (nearestPosition.equals(estimatedPosition, ABSOLUTE_ERROR)) {
                numValid++;
            }
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        final WeightedKNearestNeighboursPositionSolver3D solver =
                new WeightedKNearestNeighboursPositionSolver3D();
        try {
            solver.solve();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
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
    public void onSolveStart(final WeightedKNearestNeighboursPositionSolver<Point3D> solver) {
        solveStart++;
        checkLocked((WeightedKNearestNeighboursPositionSolver3D) solver);
    }

    @Override
    public void onSolveEnd(final WeightedKNearestNeighboursPositionSolver<Point3D> solver) {
        solveEnd++;
        checkLocked((WeightedKNearestNeighboursPositionSolver3D) solver);
    }

    private void solveKFingerprints(final int k, final double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(new Random(), 0.0, errorStd);
        }

        int numValid = 0;
        double avgDistance = 0.0;
        double avgImprovedDistance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point3D[] accessPointPositions = new Point3D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point3D[] fingerprintsPositions = new Point3D[numFingerprints];
            final List<RssiFingerprintLocated3D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated3D<>(readings, fingerprintsPositions[i]));
            }

            final RadioSourceKNearestFinder<Point3D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree3D tree = new KDTree3D(Arrays.asList(fingerprintsPositions));

            // generate measurement at random position
            final Point3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
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

            // find nearest fingerprints
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>> nearestFingerprintsList =
                    new ArrayList<>();
            final List<Double> nearestDistancesList = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k,
                    nearestFingerprintsList, nearestDistancesList);

            // noinspection unchecked
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            final double[] nearestDistances = new double[k];
            for (int i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            final WeightedKNearestNeighboursPositionSolver3D solver =
                    new WeightedKNearestNeighboursPositionSolver3D(
                            nearestFingerprints, nearestDistances, this);

            // solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);

            solver.solve();

            // check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);

            final Point3D nearestPosition = tree.nearestPoint(position);
            final Point3D estimatedPosition = solver.getEstimatedPosition();

            // check if estimated position is closer to the actual position than
            // nearest fingerprint
            final double distance = estimatedPosition.distanceTo(position);
            avgDistance += distance;

            if (distance <= nearestPosition.distanceTo(position)) {
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

    private void findBestK(final double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(new Random(), 0.0, errorStd);
        }

        double avgBestK = 0.0;
        double avgBestDistance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point3D[] accessPointPositions = new Point3D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            final int numFingerprints = randomizer.nextInt(
                    MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            final Point3D[] fingerprintsPositions = new Point3D[numFingerprints];
            final List<RssiFingerprintLocated3D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (int i = 0; i < numFingerprints; i++) {
                fingerprintsPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                for (int j = 0; j < numAccessPoints; j++) {
                    final double distance = fingerprintsPositions[i].distanceTo(
                            accessPointPositions[j]);
                    final double error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
                    final double rssi = powerTodBm(receivedPower(
                            transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                    readings.add(new RssiReading<>(accessPoints[j], rssi));
                }

                fingerprints.add(new RssiFingerprintLocated3D<>(readings, fingerprintsPositions[i]));
            }

            final RadioSourceKNearestFinder<Point3D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // generate measurement at random position
            final Point3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
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

            int bestK = 0;
            double bestDistance = Double.MAX_VALUE;
            for (int k = 1; k < numFingerprints; k++) {
                final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>> nearestFingerprintsList =
                        new ArrayList<>();
                final List<Double> nearestDistancesList = new ArrayList<>();
                finder.findKNearestTo(fingerprint, k,
                        nearestFingerprintsList, nearestDistancesList);

                // noinspection unchecked
                final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] nearestFingerprints =
                        new RssiFingerprintLocated[k];
                final double[] nearestDistances = new double[k];
                for (int i = 0; i < k; i++) {
                    nearestFingerprints[i] = nearestFingerprintsList.get(i);
                    nearestDistances[i] = nearestDistancesList.get(i);
                }

                final WeightedKNearestNeighboursPositionSolver3D solver =
                        new WeightedKNearestNeighboursPositionSolver3D(
                                nearestFingerprints, nearestDistances);

                solver.solve();

                final Point3D estimatedPosition = solver.getEstimatedPosition();

                final double distance = estimatedPosition.distanceTo(position);
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
        LOGGER.log(Level.INFO, "Best number of neighbours: {0}, Average error distance: {1}",
                new Object[]{avgBestK, avgBestDistance});
    }

    private void solveKFingerprintsUniformFingerprints(final int k, final double errorStd)
            throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(new Random(), 0.0, errorStd);
        }

        int numValid = 0;
        double avgDistance = 0.0;
        double avgImprovedDistance = 0.0;
        for (int t = 0; t < SHORT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point3D[] accessPointPositions = new Point3D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final List<Point3D> fingerprintsPositionsList = new ArrayList<>();
            final List<RssiFingerprintLocated3D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    for (double z = MIN_POS; z < MAX_POS; z += SEPARATION_POS) {
                        final InhomogeneousPoint3D fingerprintPosition =
                                new InhomogeneousPoint3D(x, y, z);
                        fingerprintsPositionsList.add(fingerprintPosition);

                        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                        for (int j = 0; j < numAccessPoints; j++) {
                            final double distance = fingerprintPosition.distanceTo(
                                    accessPointPositions[j]);
                            final double error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
                            final double rssi = powerTodBm(receivedPower(
                                    transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                            readings.add(new RssiReading<>(accessPoints[j], rssi));
                        }

                        fingerprints.add(new RssiFingerprintLocated3D<>(readings, fingerprintPosition));
                    }
                }
            }

            final RadioSourceKNearestFinder<Point3D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // build tree of fingerprint positions
            final KDTree3D tree = new KDTree3D(fingerprintsPositionsList);

            // generate measurement at random position
            final Point3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
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

            // find nearest fingerprints
            final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>> nearestFingerprintsList =
                    new ArrayList<>();
            final List<Double> nearestDistancesList = new ArrayList<>();
            finder.findKNearestTo(fingerprint, k,
                    nearestFingerprintsList, nearestDistancesList);

            // noinspection unchecked
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] nearestFingerprints =
                    new RssiFingerprintLocated[k];
            final double[] nearestDistances = new double[k];
            for (int i = 0; i < k; i++) {
                nearestFingerprints[i] = nearestFingerprintsList.get(i);
                nearestDistances[i] = nearestDistancesList.get(i);
            }

            final WeightedKNearestNeighboursPositionSolver3D solver =
                    new WeightedKNearestNeighboursPositionSolver3D(
                            nearestFingerprints, nearestDistances, this);

            // solve
            reset();
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 0);
            assertEquals(solveEnd, 0);

            solver.solve();

            // check
            assertTrue(solver.isReady());
            assertFalse(solver.isLocked());
            assertEquals(solveStart, 1);
            assertEquals(solveEnd, 1);

            final Point3D nearestPosition = tree.nearestPoint(position);
            final Point3D estimatedPosition = solver.getEstimatedPosition();

            // check if estimated position is closer to the actual position than
            // nearest fingerprint
            final double distance = estimatedPosition.distanceTo(position);
            avgDistance += distance;

            if (distance <= nearestPosition.distanceTo(position)) {
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

    private void findBestKUniformFingerprints(final double errorStd) throws NotReadyException, LockedException {
        GaussianRandomizer errorRandomizer = null;
        if (errorStd > 0.0) {
            errorRandomizer = new GaussianRandomizer(new Random(), 0.0, errorStd);
        }

        double avgBestK = 0.0;
        double avgBestDistance = 0.0;
        for (int t = 0; t < SHORT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final int numAccessPoints = randomizer.nextInt(MIN_AP, MAX_AP);
            final Point3D[] accessPointPositions = new Point3D[numAccessPoints];
            final double[] transmittedPower = new double[numAccessPoints];
            final WifiAccessPoint[] accessPoints = new WifiAccessPoint[numAccessPoints];
            for (int i = 0; i < numAccessPoints; i++) {
                accessPointPositions[i] = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));
                transmittedPower[i] = randomizer.nextDouble(
                        dBmToPower(MIN_RSSI),
                        dBmToPower(MAX_RSSI));
                accessPoints[i] = new WifiAccessPoint(String.valueOf(i), FREQUENCY);
            }

            // setup uniform fingerprint readings
            final List<RssiFingerprintLocated3D<WifiAccessPoint, RssiReading<WifiAccessPoint>>> fingerprints =
                    new ArrayList<>();
            for (double x = MIN_POS; x < MAX_POS; x += SEPARATION_POS) {
                for (double y = MIN_POS; y < MAX_POS; y += SEPARATION_POS) {
                    for (double z = MIN_POS; z < MAX_POS; z += SEPARATION_POS) {
                        final InhomogeneousPoint3D fingerprintPosition =
                                new InhomogeneousPoint3D(x, y, z);

                        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
                        for (int j = 0; j < numAccessPoints; j++) {
                            final double distance = fingerprintPosition.distanceTo(
                                    accessPointPositions[j]);
                            final double error = errorRandomizer != null ? errorRandomizer.nextDouble() : 0.0;
                            final double rssi = powerTodBm(receivedPower(
                                    transmittedPower[j], distance, accessPoints[j].getFrequency())) + error;
                            readings.add(new RssiReading<>(accessPoints[j], rssi));
                        }

                        fingerprints.add(new RssiFingerprintLocated3D<>(readings, fingerprintPosition));
                    }
                }
            }

            final RadioSourceKNearestFinder<Point3D, WifiAccessPoint> finder =
                    new RadioSourceKNearestFinder<>(fingerprints);

            // generate measurement at random position
            final Point3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
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

            int bestK = 0;
            double bestDistance = Double.MAX_VALUE;
            int numFingerprints = fingerprints.size();
            int maxK = Math.min(numFingerprints, MAX_K);
            for (int k = 1; k < maxK; k++) {
                final List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>> nearestFingerprintsList =
                        new ArrayList<>();
                final List<Double> nearestDistancesList = new ArrayList<>();
                finder.findKNearestTo(fingerprint, k,
                        nearestFingerprintsList, nearestDistancesList);

                // noinspection unchecked
                final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, Point3D>[] nearestFingerprints =
                        new RssiFingerprintLocated[k];
                final double[] nearestDistances = new double[k];
                for (int i = 0; i < k; i++) {
                    nearestFingerprints[i] = nearestFingerprintsList.get(i);
                    nearestDistances[i] = nearestDistancesList.get(i);
                }

                final WeightedKNearestNeighboursPositionSolver3D solver =
                        new WeightedKNearestNeighboursPositionSolver3D(
                                nearestFingerprints, nearestDistances);

                solver.solve();

                final Point3D estimatedPosition = solver.getEstimatedPosition();

                final double distance = estimatedPosition.distanceTo(position);
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

    private void checkLocked(final WeightedKNearestNeighboursPositionSolver3D solver) {
        try {
            solver.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setFingerprintsAndDistances(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.setEpsilon(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            solver.solve();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
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
