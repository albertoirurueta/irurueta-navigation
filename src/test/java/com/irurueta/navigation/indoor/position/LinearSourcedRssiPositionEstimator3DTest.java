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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class LinearSourcedRssiPositionEstimator3DTest implements SourcedRssiPositionEstimatorListener<Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            LinearSourcedRssiPositionEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final int MIN_FINGERPRINTS = 100;
    private static final int MAX_FINGERPRINTS = 1000;

    private static final double RSSI_BIAS = 1.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double ERROR_STD = 1.0e-1;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    public LinearSourcedRssiPositionEstimator3DTest() { }

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
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertTrue(estimator.isMeansFromFingerprintReadingsRemoved());


        //test constructor with listener
        estimator = new LinearSourcedRssiPositionEstimator3D(this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertTrue(estimator.isMeansFromFingerprintReadingsRemoved());


        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RssiReading<RadioSource>> readings = new ArrayList<>();
        for (int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i,
                    FREQUENCY);
            double rssi = randomizer.nextDouble();

            RssiReading<RadioSource> reading =
                    new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        RssiFingerprintLocated3D<RadioSource,
                RssiReading<RadioSource>>
                locatedFingerprint = new RssiFingerprintLocated3D<>(readings,
                Point3D.create());

        List<RssiFingerprintLocated3D<RadioSource,
                RssiReading<RadioSource>>> locatedFingerprints =
                new ArrayList<>();
        locatedFingerprints.add(locatedFingerprint);


        RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);


        List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
        for (int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            WifiAccessPointLocated3D source = new WifiAccessPointLocated3D("bssid" + 1,
                    FREQUENCY, Point3D.create());
            sources.add(source);
        }


        //test constructor with located fingerprints, fingerprint and sources
        estimator = new LinearSourcedRssiPositionEstimator3D(locatedFingerprints,
                fingerprint, sources);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertTrue(estimator.isMeansFromFingerprintReadingsRemoved());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearSourcedRssiPositionEstimator3D(new ArrayList<RssiFingerprintLocated3D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearSourcedRssiPositionEstimator3D(null,
                    fingerprint, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearSourcedRssiPositionEstimator3D(locatedFingerprints,
                    null, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearSourcedRssiPositionEstimator3D(locatedFingerprints,
                    fingerprint, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, sources and listener
        estimator = new LinearSourcedRssiPositionEstimator3D(locatedFingerprints,
                fingerprint, sources, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 3);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertTrue(estimator.isMeansFromFingerprintReadingsRemoved());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LinearSourcedRssiPositionEstimator3D(new ArrayList<RssiFingerprintLocated3D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearSourcedRssiPositionEstimator3D(null,
                    fingerprint, sources,this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearSourcedRssiPositionEstimator3D(locatedFingerprints,
                    null, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new LinearSourcedRssiPositionEstimator3D(locatedFingerprints,
                    fingerprint, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetLocatedFingerprints() throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default value
        assertNull(estimator.getLocatedFingerprints());

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RssiReading<RadioSource>> readings = new ArrayList<>();
        for (int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i,
                    FREQUENCY);
            double rssi = randomizer.nextDouble();

            RssiReading<RadioSource> reading =
                    new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        RssiFingerprintLocated3D<RadioSource,
                RssiReading<RadioSource>>
                locatedFingerprint = new RssiFingerprintLocated3D<>(readings,
                Point3D.create());

        List<RssiFingerprintLocated3D<RadioSource,
                RssiReading<RadioSource>>> locatedFingerprints =
                new ArrayList<>();
        locatedFingerprints.add(locatedFingerprint);

        estimator.setLocatedFingerprints(locatedFingerprints);

        //check correctness
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);

        //force IllegalArgumentException
        try {
            estimator.setLocatedFingerprints(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setLocatedFingerprints(new ArrayList<RssiFingerprintLocated<RadioSource,
                    RssiReading<RadioSource>, Point3D>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default value
        assertNull(estimator.getFingerprint());

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RssiReading<RadioSource>> readings = new ArrayList<>();
        for (int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i,
                    FREQUENCY);
            double rssi = randomizer.nextDouble();

            RssiReading<RadioSource> reading =
                    new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);
        estimator.setFingerprint(fingerprint);

        //check correctness
        assertSame(estimator.getFingerprint(), fingerprint);

        //force IllegalArgumentException
        try {
            estimator.setFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetMinMaxNearestFingerprints() throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default values
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);

        //set new values
        estimator.setMinMaxNearestFingerprints(2, 3);

        //check
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);

        //force IllegalArgumentException
        try {
            estimator.setMinMaxNearestFingerprints(0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setMinMaxNearestFingerprints(2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetPathLossExponent() throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default value
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double pathLossExponent = randomizer.nextDouble();

        estimator.setPathLossExponent(pathLossExponent);

        //check correctness
        assertEquals(estimator.getPathLossExponent(), pathLossExponent, 0.0);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetSources() throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default values
        assertNull(estimator.getSources());

        //set new value
        List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
        estimator.setSources(sources);

        //check
        assertSame(estimator.getSources(), sources);

        //force IllegalArgumentException
        try {
            estimator.setSources(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetUseSourcesPathLossExponentWhenAvailable()
            throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default value
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //set new value
        estimator.setUseSourcesPathLossExponentWhenAvailable(false);

        //check
        assertFalse(estimator.getUseSourcesPathLossExponentWhenAvailable());
    }

    @Test
    public void testGetSetUseNoMeanNearestFingerprintFinder()
            throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default value
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());

        //set new value
        estimator.setUseNoMeanNearestFingerprintFinder(false);

        //check
        assertFalse(estimator.getUseNoMeanNearestFingerprintFinder());
    }

    @Test
    public void testIsSetMeansFromFingerprintReadingsRemoved()
            throws LockedException {
        LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D();

        //check default value
        assertTrue(estimator.isMeansFromFingerprintReadingsRemoved());

        //set new value
        estimator.setMeansFromFingerprintReadingsRemoved(false);

        //check
        assertFalse(estimator.isMeansFromFingerprintReadingsRemoved());
    }

    @Test
    public void testEstimateWithoutErrorAndWithoutBias() throws LockedException,
            NotReadyException,
            PositionEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point3D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            //build non-located fingerprint
            double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            double z = randomizer.nextDouble(MIN_POS, MAX_POS);
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point3D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find real closest fingerprint based on location
            RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point3D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                Point3D fingerprintPosition = locatedFingerprint.getPosition();
                double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;


            //find closest fingerprint based on RSSI without mean
            RadioSourceNoMeanKNearestFinder<Point3D, RadioSource> noMeanfinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprintNoMean =
                    noMeanfinder.findNearestTo(fingerprint);
            Point3D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;


            //find closest fingerprint based on RSSI
            RadioSourceKNearestFinder<Point3D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            Point3D rssiClosestPosition = nearestFingerprint.getPosition();

            double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;


            //create estimator with means removed on finder and fingerprints
            LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on finder
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on readings
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means not removed
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);

            double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            //find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }

        }

        LOGGER.log(Level.INFO, "Results without error and without bias");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double)numBestIsNoMeanRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double)numBestIsRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double)numBestIsNoMeansEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double)numBestIsNoMeanFinderEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double)numBestIsNoMeanReadingsEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double)numBestIsEstimatedPosition / (double)TIMES * 100.0);


        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m",
                avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m",
                avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m",
                avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m",
                avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        //check that best result is obtained when means are removed to find
        //closest fingerprints, but not removed for readings
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertEquals(bestNum, numBestIsNoMeanReadingsEstimatedPosition);
        assertEquals(bestPos, 4);
    }

    @Test
    public void testEstimateWithBias() throws LockedException,
            NotReadyException,
            PositionEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point3D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            //build non-located fingerprint
            double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            double z = randomizer.nextDouble(MIN_POS, MAX_POS);
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point3D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT)) +
                        RSSI_BIAS;
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find real closest fingerprint based on location
            RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point3D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                Point3D fingerprintPosition = locatedFingerprint.getPosition();
                double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;


            //find closest fingerprint based on RSSI without mean
            RadioSourceNoMeanKNearestFinder<Point3D, RadioSource> noMeanfinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprintNoMean =
                    noMeanfinder.findNearestTo(fingerprint);
            Point3D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;


            //find closest fingerprint based on RSSI
            RadioSourceKNearestFinder<Point3D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            Point3D rssiClosestPosition = nearestFingerprint.getPosition();

            double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;


            //create estimator with means removed on finder and fingerprints
            LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on finder
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on readings
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means not removed
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);

            double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            //find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }

        }

        LOGGER.log(Level.INFO, "Results with bias");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double)numBestIsNoMeanRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double)numBestIsRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double)numBestIsNoMeansEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double)numBestIsNoMeanFinderEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double)numBestIsNoMeanReadingsEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double)numBestIsEstimatedPosition / (double)TIMES * 100.0);


        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m",
                avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m",
                avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m",
                avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m",
                avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        //check that best result is obtained when means are removed from both
        //fingerprint finder and readings to account for possible biases between devices
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertTrue(bestNum == numBestIsNoMeanRssiPosition ||
                bestNum == numBestIsNoMeanFinderEstimatedPosition);
        assertTrue(bestPos == 0 || bestPos == 3);
    }

    @Test
    public void testEstimateWithError() throws LockedException,
            NotReadyException,
            PositionEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point3D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                    double rssiError = errorRandomizer.nextDouble();
                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi + rssiError);
                    readings.add(reading);
                }

                RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            //build non-located fingerprint
            double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            double z = randomizer.nextDouble(MIN_POS, MAX_POS);
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point3D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                double rssiError = errorRandomizer.nextDouble();
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi + rssiError);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find real closest fingerprint based on location
            RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point3D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                Point3D fingerprintPosition = locatedFingerprint.getPosition();
                double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;


            //find closest fingerprint based on RSSI without mean
            RadioSourceNoMeanKNearestFinder<Point3D, RadioSource> noMeanfinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprintNoMean =
                    noMeanfinder.findNearestTo(fingerprint);
            Point3D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;


            //find closest fingerprint based on RSSI
            RadioSourceKNearestFinder<Point3D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            Point3D rssiClosestPosition = nearestFingerprint.getPosition();

            double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;


            //create estimator with means removed on finder and fingerprints
            LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on finder
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on readings
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means not removed
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);

            double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            //find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }

        }

        LOGGER.log(Level.INFO, "Results with error bias");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double)numBestIsNoMeanRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double)numBestIsRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double)numBestIsNoMeansEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double)numBestIsNoMeanFinderEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double)numBestIsNoMeanReadingsEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double)numBestIsEstimatedPosition / (double)TIMES * 100.0);


        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m",
                avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m",
                avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m",
                avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m",
                avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        //check that best result is obtained when means are removed from both
        //fingerprint finder and readings to account for possible biases between devices
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertEquals(bestNum, numBestIsNoMeanReadingsEstimatedPosition);
        assertEquals(bestPos, 4);
    }

    @Test
    public void testEstimateWithErrorAndWithBias() throws LockedException,
            NotReadyException,
            PositionEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(new Random(),
                    0.0, ERROR_STD);

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point3D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                    double rssiError = errorRandomizer.nextDouble();
                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi + rssiError);
                    readings.add(reading);
                }

                RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            //build non-located fingerprint
            double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            double z = randomizer.nextDouble(MIN_POS, MAX_POS);
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point3D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                double rssiError = errorRandomizer.nextDouble();
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi + rssiError + RSSI_BIAS);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find real closest fingerprint based on location
            RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point3D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                Point3D fingerprintPosition = locatedFingerprint.getPosition();
                double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;


            //find closest fingerprint based on RSSI without mean
            RadioSourceNoMeanKNearestFinder<Point3D, RadioSource> noMeanfinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprintNoMean =
                    noMeanfinder.findNearestTo(fingerprint);
            Point3D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;


            //find closest fingerprint based on RSSI
            RadioSourceKNearestFinder<Point3D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            Point3D rssiClosestPosition = nearestFingerprint.getPosition();

            double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;


            //create estimator with means removed on finder and fingerprints
            LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on finder
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on readings
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means not removed
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);

            double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            //find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }

        }

        LOGGER.log(Level.INFO, "Results with error and bias");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double)numBestIsNoMeanRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double)numBestIsRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double)numBestIsNoMeansEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double)numBestIsNoMeanFinderEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double)numBestIsNoMeanReadingsEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double)numBestIsEstimatedPosition / (double)TIMES * 100.0);


        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m",
                avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m",
                avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m",
                avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m",
                avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        //check that best result is obtained when means are removed from both
        //fingerprint finder and readings to account for possible biases between devices
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertTrue(bestNum == numBestIsNoMeanRssiPosition ||
                bestNum == numBestIsNoMeanFinderEstimatedPosition);
        assertTrue(bestPos == 0 || bestPos == 3);

    }

    @Test
    public void testEstimateWithOtherPathloss() throws LockedException,
            NotReadyException,
            PositionEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);

                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm,
                                position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point3D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, pathLossExponent));
                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            //build non-located fingerprint
            double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            double z = randomizer.nextDouble(MIN_POS, MAX_POS);
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point3D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        pathLossExponent));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find real closest fingerprint based on location
            RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point3D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                Point3D fingerprintPosition = locatedFingerprint.getPosition();
                double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;


            //find closest fingerprint based on RSSI without mean
            RadioSourceNoMeanKNearestFinder<Point3D, RadioSource> noMeanfinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprintNoMean =
                    noMeanfinder.findNearestTo(fingerprint);
            Point3D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;


            //find closest fingerprint based on RSSI
            RadioSourceKNearestFinder<Point3D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            Point3D rssiClosestPosition = nearestFingerprint.getPosition();

            double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;


            //create estimator with means removed on finder and fingerprints
            LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);
            estimator.setPathLossExponent(pathLossExponent);
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on finder
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);
            estimator.setPathLossExponent(pathLossExponent);
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means removed only on readings
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);
            estimator.setPathLossExponent(pathLossExponent);
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);


            //create estimator with means not removed
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);
            estimator.setPathLossExponent(pathLossExponent);
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 1);

            double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            //find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }

        }

        LOGGER.log(Level.INFO, "Results for different path loss exponents");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double)numBestIsNoMeanRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double)numBestIsRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double)numBestIsNoMeansEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double)numBestIsNoMeanFinderEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double)numBestIsNoMeanReadingsEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double)numBestIsEstimatedPosition / (double)TIMES * 100.0);


        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m",
                avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m",
                avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m",
                avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m",
                avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);

        int[] numBest = new int[]{
                numBestIsNoMeanRssiPosition,
                numBestIsRssiPosition,
                numBestIsNoMeansEstimatedPosition,
                numBestIsNoMeanFinderEstimatedPosition,
                numBestIsNoMeanReadingsEstimatedPosition,
                numBestIsEstimatedPosition
        };

        //check that best result is obtained when means are removed to find
        //closest fingerprints, but not removed for readings
        int bestNum = -Integer.MAX_VALUE;
        int bestPos = -1;
        for (int i = 0; i < numBest.length; i++) {
            if (numBest[i] > bestNum) {
                bestNum = numBest[i];
                bestPos = i;
            }
        }

        assertEquals(bestNum, numBestIsNoMeanReadingsEstimatedPosition);
        assertEquals(bestPos, 4);
    }

    @Test
    public void testEstimateWithoutErrorAndWithoutBiasOneRadioSource() throws LockedException,
            NotReadyException, PositionEstimationException {
        int numBestIsNoMeanRssiPosition = 0;
        int numBestIsRssiPosition = 0;
        int numBestIsNoMeansEstimatedPosition = 0;
        int numBestIsNoMeanFinderEstimatedPosition = 0;
        int numBestIsNoMeanReadingsEstimatedPosition = 0;
        int numBestIsEstimatedPosition = 0;

        double avgClosestDistance = 0.0;
        double avgNoMeanRssiDistance = 0.0;
        double avgRssiDistance = 0.0;
        double avgNoMeansEstimatedError = 0.0;
        double avgNoMeanFinderEstimatedError = 0.0;
        double avgNoMeanReadingsEstimatedError = 0.0;
        double avgEstimatedError = 0.0;

        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point3D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            //build non-located fingerprint
            double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            double z = randomizer.nextDouble(MIN_POS, MAX_POS);
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point3D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance,
                        LinearSourcedRssiPositionEstimator3D.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find real closest fingerprint based on location
            RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> closestFingerprint = null;
            Point3D closestPosition = null;
            double distance = Double.MAX_VALUE;
            for (RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint :
                    locatedFingerprints) {
                Point3D fingerprintPosition = locatedFingerprint.getPosition();
                double dist = fingerprintPosition.distanceTo(position);
                if (dist < distance) {
                    distance = dist;
                    closestFingerprint = locatedFingerprint;
                    closestPosition = fingerprintPosition;
                }
            }

            assertNotNull(closestFingerprint);
            assertNotNull(closestPosition);

            double closestDistance = closestPosition.distanceTo(position);
            avgClosestDistance += closestDistance / TIMES;


            //find closest fingerprint based on RSSI without mean
            RadioSourceNoMeanKNearestFinder<Point3D, RadioSource> noMeanfinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprintNoMean =
                    noMeanfinder.findNearestTo(fingerprint);
            Point3D noMeanRssiClosestPosition = nearestFingerprintNoMean.getPosition();

            double noMeanRssiClosestDistance = noMeanRssiClosestPosition.distanceTo(position);
            avgNoMeanRssiDistance += noMeanRssiClosestDistance / TIMES;


            //find closest fingerprint based on RSSI
            RadioSourceKNearestFinder<Point3D, RadioSource> finder =
                    new RadioSourceKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearestFingerprint =
                    finder.findNearestTo(fingerprint);
            Point3D rssiClosestPosition = nearestFingerprint.getPosition();

            double rssiClosestDistance = rssiClosestPosition.distanceTo(position);
            avgRssiDistance += rssiClosestDistance / TIMES;


            //create estimator with means removed on finder and fingerprints
            LinearSourcedRssiPositionEstimator3D estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeansEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeansEstimatedError += noMeansEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 3);


            //create estimator with means removed only on finder
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanFinderEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanFinderEstimatedError += noMeanFinderEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 3);


            //create estimator with means removed only on readings
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double noMeanReadingsEstimatedError = estimatedPosition.distanceTo(position);
            avgNoMeanReadingsEstimatedError += noMeanReadingsEstimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 3);


            //create estimator with means not removed
            estimator = new LinearSourcedRssiPositionEstimator3D(
                    locatedFingerprints, fingerprint, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            estimator.setMeansFromFingerprintReadingsRemoved(false);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check correctness
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            estimatedPosition = estimator.getEstimatedPosition();

            assertNotNull(estimatedPosition);

            double estimatedError = estimatedPosition.distanceTo(position);
            avgEstimatedError += estimatedError / TIMES;

            assertNotNull(estimator.getNearestFingerprints());
            assertEquals(estimator.getNearestFingerprints().size(), 3);

            double[] errors = new double[]{
                    closestDistance,
                    noMeanRssiClosestDistance,
                    rssiClosestDistance,
                    noMeansEstimatedError,
                    noMeanFinderEstimatedError,
                    noMeanReadingsEstimatedError,
                    estimatedError
            };

            //find minimum error
            double minError = Double.MAX_VALUE;
            int bestErrorPos = -1;
            for (int i = 0; i < errors.length; i++) {
                if (errors[i] < minError) {
                    minError = errors[i];
                    bestErrorPos = i;
                }
            }

            switch (bestErrorPos) {
                case 0:
                    numBestIsNoMeanRssiPosition++;
                    break;
                case 1:
                    numBestIsRssiPosition++;
                    break;
                case 2:
                    numBestIsNoMeansEstimatedPosition++;
                    break;
                case 3:
                    numBestIsNoMeanFinderEstimatedPosition++;
                    break;
                case 4:
                    numBestIsNoMeanReadingsEstimatedPosition++;
                    break;
                case 5:
                    numBestIsEstimatedPosition++;
                    break;
            }

        }

        LOGGER.log(Level.INFO, "Results without error and without bias with a single radio source");

        LOGGER.log(Level.INFO, "Percentage best no mean RSSI: {0}%",
                (double)numBestIsNoMeanRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best RSSI: {0}%",
                (double)numBestIsRssiPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no means: {0}%",
                (double)numBestIsNoMeansEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean finder: {0}%",
                (double)numBestIsNoMeanFinderEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated no mean readings: {0}%",
                (double)numBestIsNoMeanReadingsEstimatedPosition / (double)TIMES * 100.0);
        LOGGER.log(Level.INFO, "Percentage best estimated: {0}%",
                (double)numBestIsEstimatedPosition / (double)TIMES * 100.0);


        LOGGER.log(Level.INFO, "Avg. closest fingerprint distance: {0} m",
                avgClosestDistance);
        LOGGER.log(Level.INFO,
                "Avg. no mean RSSI closest fingerprint distance: {0} m",
                avgNoMeanRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. RSSI closest fingerprint distance: {0} m",
                avgRssiDistance);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position error with means removed: {0} m",
                avgNoMeansEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on finder error: {0} m",
                avgNoMeanFinderEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means removed only on readings error: {0} m",
                avgNoMeanReadingsEstimatedError);
        LOGGER.log(Level.INFO,
                "Avg. Estimated position with means not removed error: {0} m",
                avgEstimatedError);
    }

    @Override
    public void onEstimateStart(SourcedRssiPositionEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((LinearSourcedRssiPositionEstimator3D)estimator);
    }

    @Override
    public void onEstimateEnd(SourcedRssiPositionEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((LinearSourcedRssiPositionEstimator3D)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
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

    private void checkLocked(LinearSourcedRssiPositionEstimator3D estimator) {
        try {
            estimator.setLocatedFingerprints(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMinMaxNearestFingerprints(1, 1);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPathLossExponent(2.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setUseSourcesPathLossExponentWhenAvailable(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setUseNoMeanNearestFingerprintFinder(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMeansFromFingerprintReadingsRemoved(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
