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

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class SourcedRssiPositionEstimator2DTest implements SourcedRssiPositionEstimatorListener<Point2D> {

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

    private static final double ABSOLUTE_ERROR = 1e-6;
//    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

//    private static final double ERROR_STD = 1e-3;

    private static final int TIMES = 10;

    private int estimateStart;
    private int estimateEnd;

    public SourcedRssiPositionEstimator2DTest() { }

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
        SourcedRssiPositionEstimator2D estimator = new SourcedRssiPositionEstimator2D();

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());


        //test constructor with listener
        estimator = new SourcedRssiPositionEstimator2D(this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());


        //test constructor with located fingerprints
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        List<RssiReading<RadioSource>> readings = new ArrayList<>();
        for (int i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i,
                    FREQUENCY);
            double rssi = randomizer.nextDouble();

            RssiReading<RadioSource> reading =
                    new RssiReading<>((RadioSource) accessPoint, rssi);
            readings.add(reading);
        }

        RssiFingerprintLocated2D<RadioSource,
                RssiReading<RadioSource>>
                locatedFingerprint = new RssiFingerprintLocated2D<>(readings,
                Point2D.create());

        List<RssiFingerprintLocated2D<RadioSource,
                RssiReading<RadioSource>>> locatedFingerprints =
                new ArrayList<>();
        locatedFingerprints.add(locatedFingerprint);

        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint
        RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);
        estimator = new SourcedRssiPositionEstimator2D(fingerprint);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint and listener
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints and fingerprint
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, fingerprint);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with min/max values
        estimator = new SourcedRssiPositionEstimator2D(2, 3);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //test constructor without max limit
        estimator = new SourcedRssiPositionEstimator2D(2, -1);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);

        //test constructor with min/max values and listener
        estimator = new SourcedRssiPositionEstimator2D(2,
                3, this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //test constructor without max limit
        estimator = new SourcedRssiPositionEstimator2D(2,
                -1, this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(0, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(2, 1, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints and min/max values
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                2, 3);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), 2,
                    3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, 2,
                    3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, min/max values and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                2, 3, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), 2,
                    3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, 2,
                    3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    0, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    2, 1, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint and min/max values
        estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                2, 3);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, min/max values and listener
        estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                2, 3, this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    2, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    0, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    2, 1, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and min/max values
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, 2, 3);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, 2,
                    3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, 2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>>) null,
                    2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, 0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, 2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, min/max values and
        //listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, 2, 3, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, 2,
                    3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, 2, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>>) null,
                    2, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, 0, 3,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, 2, 1,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);



        //test constructor with path loss
        double pathLossExponent = randomizer.nextDouble();

        estimator = new SourcedRssiPositionEstimator2D(pathLossExponent);


        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());


        //test constructor with path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(pathLossExponent, this);


        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());


        //test constructor with located fingerprints and path loss
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                pathLossExponent);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                pathLossExponent, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint and path loss
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, pathLossExponent);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, pathLossExponent,
                this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and path loss
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, fingerprint,
                pathLossExponent);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, path loss and
        //listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, pathLossExponent, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with min/max values
        estimator = new SourcedRssiPositionEstimator2D(2,
                3, pathLossExponent);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(),  pathLossExponent, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //test constructor without max limit
        estimator = new SourcedRssiPositionEstimator2D(2,
                -1, pathLossExponent);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(0,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(2,
                    1, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);

        //test constructor with min/max values, path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(2,
                3, pathLossExponent, this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //test constructor without max limit
        estimator = new SourcedRssiPositionEstimator2D(2,
                -1, pathLossExponent, this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(0,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(2,
                    1, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, min/max values and path loss
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                2, 3, pathLossExponent);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), 2,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, 2,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    0, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    2, 1, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, min/max values, path loss
        //and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                2, 3, pathLossExponent,
                this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), 2,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, 2,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    0, 3, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    2, 1, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, min/max values and path loss
        estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                2, 3, pathLossExponent);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    2, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    0, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    2, 1, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, min/max values, path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                2, 3, pathLossExponent,
                this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    2, 3, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    0, 3, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    2, 1, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, min/max values and
        //path loss
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, 2, 3,
                pathLossExponent);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, 2,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, 2, 3,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>>) null,
                    2, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, 0, 3,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, 2, 1,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, min/max values,
        //path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, 2, 3,
                pathLossExponent, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getSources());
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, 2,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, 2, 3,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (RssiFingerprint<? extends RadioSource, ? extends RssiReading<? extends RadioSource>>) null,
                    2, 3, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, 0, 3,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, 2, 1,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints and sources
        List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
        for (int i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            WifiAccessPointLocated2D source = new WifiAccessPointLocated2D("bssid" + 1,
                    FREQUENCY, Point2D.create());
            sources.add(source);
        }

        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, sources);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (List<? extends RadioSourceLocated<Point2D>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, sources and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, sources,
                this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (List<? extends RadioSourceLocated<Point2D>>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint and sources
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint ,
                    (List<? extends RadioSourceLocated<Point2D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, sources and listener
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and sources
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, sources);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    null, sources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, (List<? extends RadioSourceLocated<Point2D>>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, sources and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, sources, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, sources,this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    null, sources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, sources and min/max values
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                sources, 2, 3);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), sources, 2,
                    3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, sources, 2,
                    3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    sources, 0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    sources, 2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (List<? extends RadioSourceLocated<Point2D>>)null,
                    2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, sources, min/max values and
        //listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                sources, 2, 3, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), sources, 2,
                    3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, sources, 2,
                    3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    sources, 0, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    sources, 2, 1, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (List<? extends RadioSourceLocated<Point2D>>)null, 2, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, sources and min/max values
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                2, 3);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    sources, 2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint, null,
                    2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    sources, 0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    sources, 2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, sources, min/max values and listener
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                2, 3, this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    sources, 2, 3,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    null, 2, 3,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                    0, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                    2, 1, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and min/max values
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, sources,2, 3);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources,
                    2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, sources, 2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    null, sources, 2, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, null, 2,
                    3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, sources, 0, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, sources, 2, 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, min/max values and
        //listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, sources, 2, 3,
                this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources,
                    2, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, sources, 2,
                    3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    null, sources, 2, 3,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, null, 2, 3, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, sources, 0, 3,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, sources, 2, 1,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, sources and path loss
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                sources, pathLossExponent);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), sources, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, sources, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (List<? extends RadioSourceLocated<Point2D>>)null,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, sources, path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                sources, pathLossExponent, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), sources, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, sources, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (List<? extends RadioSourceLocated<Point2D>>)null,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, sources and path loss
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                pathLossExponent);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    sources, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint, null,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, sources, path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                pathLossExponent, this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    sources, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    null, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, sources and path loss
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, fingerprint,
                sources, pathLossExponent);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, sources, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    null, sources, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, null, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, sources, path loss
        //and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, sources, pathLossExponent, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, sources, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    null, sources, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, null, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, sources, min/max values and
        //path loss
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, sources,
                2, 3, pathLossExponent);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), sources, 2,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, sources, 2,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, sources,
                    0, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, sources,
                    2, 1, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (List<? extends RadioSourceLocated<Point2D>>)null,
                    2, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, sources, min/max values, path loss
        //and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, sources,
                2, 3, pathLossExponent,
                this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), sources, 2,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D((List<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>)null, sources, 2,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, sources,
                    0, 3, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints, sources,
                    2, 1, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    (List<? extends RadioSourceLocated<Point2D>>)null,
                    2, 3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, sources, min/max values and path loss
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                2, 3, pathLossExponent);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    sources, 2, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    null, 2, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    sources, 0, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    sources, 2, 1, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with fingerprint, sources, min/max values, path loss and
        //listener
        estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                2, 3, pathLossExponent,
                this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertFalse(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(
                    (RssiFingerprint<RadioSource, RssiReading<RadioSource>>)null,
                    sources, 2, 3,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint,
                    null, 2, 3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                    0, 3, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(fingerprint, sources,
                    2, 1, pathLossExponent,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, sources, min/max
        //values and path loss
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, sources, 2, 3,
                pathLossExponent);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources, 2,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, sources, 2, 3,
                    pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    null, sources, 2,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, null, 2,
                    3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, sources, 0, 3, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, sources, 2, 1, pathLossExponent);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, sources, min/max
        //values, path loss and listener
        estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                fingerprint, sources, 2, 3,
                pathLossExponent, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 2);
        assertEquals(estimator.getMaxNearestFingerprints(), 3);
        assertEquals(estimator.getPathLossExponent(), pathLossExponent,
                0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.isReady());
        assertNull(estimator.getEstimatedPosition());
        assertSame(estimator.getSources(), sources);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new SourcedRssiPositionEstimator2D(new ArrayList<RssiFingerprintLocated2D<RadioSource,
                    RssiReading<RadioSource>>>(), fingerprint, sources,
                    2, 3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(null,
                    fingerprint, sources, 2,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    null, sources, 2,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, null, 2,
                    3, pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, sources, 0, 3,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new SourcedRssiPositionEstimator2D(locatedFingerprints,
                    fingerprint, sources, 2, 1,
                    pathLossExponent, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Override
    public void onEstimateStart(SourcedRssiPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((SourcedRssiPositionEstimator2D)estimator);
    }

    @Override
    public void onEstimateEnd(SourcedRssiPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((SourcedRssiPositionEstimator2D)estimator);
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

    private void checkLocked(SourcedRssiPositionEstimator2D estimator) {
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
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
