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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RangingFingerprint;
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated3D;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class LinearRangingPositionEstimator3DTest implements
        RangingPositionEstimatorListener<Point3D> {

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 4;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final double ERROR_STD = 1e-3;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;

    @Test
    public void testConstructor() {
        // empty constructor
        LinearRangingPositionEstimator3D estimator =
                new LinearRangingPositionEstimator3D();

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());


        // constructor with sources
        List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }
        estimator = new LinearRangingPositionEstimator3D(sources);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRangingPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearRangingPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprint
        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        estimator = new LinearRangingPositionEstimator3D(fingerprint);

        // check default value
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRangingPositionEstimator3D(
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources and fingerprint
        estimator = new LinearRangingPositionEstimator3D(sources, fingerprint);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRangingPositionEstimator3D(null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearRangingPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearRangingPositionEstimator3D(sources,
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with listener
        estimator = new LinearRangingPositionEstimator3D(this);

        // check
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());


        // constructor with sources and listener
        estimator = new LinearRangingPositionEstimator3D(sources, this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRangingPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearRangingPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprint and listener
        estimator = new LinearRangingPositionEstimator3D(fingerprint, this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRangingPositionEstimator3D(
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources, fingerprint and listener
        estimator = new LinearRangingPositionEstimator3D(sources, fingerprint,
                this);

        // check default values
        assertNull(estimator.getEstimatedPosition());
        assertTrue(estimator.isHomogeneousLinearSolverUsed());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearRangingPositionEstimator3D(null, fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearRangingPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearRangingPositionEstimator3D(sources, null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetSources() throws LockedException {
        LinearRangingPositionEstimator3D estimator =
                new LinearRangingPositionEstimator3D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }

        estimator.setSources(sources);

        // check
        assertSame(estimator.getSources(), sources);

        // force IllegalArgumentException
        try {
            estimator.setSources(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setSources(new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        LinearRangingPositionEstimator3D estimator =
                new LinearRangingPositionEstimator3D();

        // check default value
        assertNull(estimator.getFingerprint());

        // set new value
        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        estimator.setFingerprint(fingerprint);

        // check
        assertSame(estimator.getFingerprint(), fingerprint);

        // force IllegalArgumentException
        try {
            estimator.setFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        LinearRangingPositionEstimator3D estimator =
                new LinearRangingPositionEstimator3D();

        // check default size
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateNoErrorHomogeneous() throws LockedException, NotReadyException,
            PositionEstimationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid,
                                FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            LinearRangingPositionEstimator3D estimator =
                    new LinearRangingPositionEstimator3D(sources, fingerprint,
                            this);

            reset();

            // check initial state
            assertTrue(estimator.isHomogeneousLinearSolverUsed());
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }

        // force NotReadyException
        LinearRangingPositionEstimator3D estimator =
                new LinearRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithErrorHomogeneous() throws LockedException, NotReadyException,
            PositionEstimationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                0.0, ERROR_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid,
                                FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double error = errorRandomizer.nextDouble();

                readings.add(new RangingReading<>(accessPoint,
                        distance + error));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            LinearRangingPositionEstimator3D estimator =
                    new LinearRangingPositionEstimator3D(sources, fingerprint,
                            this);

            reset();

            // check initial state
            assertTrue(estimator.isHomogeneousLinearSolverUsed());
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            double distance = position.distanceTo(estimatedPosition);
            if (distance >= LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            numValid++;
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        LinearRangingPositionEstimator3D estimator =
                new LinearRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateNoErrorInhomogeneous() throws LockedException, NotReadyException,
            PositionEstimationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid,
                                FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                readings.add(new RangingReading<>(accessPoint, distance));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            LinearRangingPositionEstimator3D estimator =
                    new LinearRangingPositionEstimator3D(sources, fingerprint,
                            this);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();

            // check initial state
            assertFalse(estimator.isHomogeneousLinearSolverUsed());
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
        }

        // force NotReadyException
        LinearRangingPositionEstimator3D estimator =
                new LinearRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateWithErrorInhomogeneous() throws LockedException, NotReadyException,
            PositionEstimationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                0.0, ERROR_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));

            List<WifiAccessPointLocated3D> sources = new ArrayList<>();
            List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                String bssid = String.valueOf(i);

                WifiAccessPointLocated3D locatedAccessPoint =
                        new WifiAccessPointLocated3D(bssid,
                                FREQUENCY, accessPointPosition);
                sources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint(bssid, FREQUENCY);

                double distance = position.distanceTo(accessPointPosition);

                double error = errorRandomizer.nextDouble();

                readings.add(new RangingReading<>(accessPoint,
                        distance + error));
            }

            RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                    new RangingFingerprint<>(readings);

            LinearRangingPositionEstimator3D estimator =
                    new LinearRangingPositionEstimator3D(sources, fingerprint,
                            this);
            estimator.setHomogeneousLinearSolverUsed(false);

            reset();

            // check initial state
            assertFalse(estimator.isHomogeneousLinearSolverUsed());
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            double distance = position.distanceTo(estimatedPosition);
            if (distance >= LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            numValid++;
            assertTrue(position.equals(estimatedPosition, LARGE_ABSOLUTE_ERROR));
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        LinearRangingPositionEstimator3D estimator =
                new LinearRangingPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(RangingPositionEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((LinearRangingPositionEstimator3D) estimator);
    }

    @Override
    public void onEstimateEnd(RangingPositionEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((LinearRangingPositionEstimator3D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void checkLocked(LinearRangingPositionEstimator3D estimators) {
        try {
            estimators.setHomogeneousLinearSolverUsed(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimators.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimators.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimators.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimators.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
