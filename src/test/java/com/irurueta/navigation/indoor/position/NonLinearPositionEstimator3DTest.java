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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class NonLinearPositionEstimator3DTest implements PositionEstimatorListener<Point3D> {

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 4;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    public NonLinearPositionEstimator3DTest() { }

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
        //empty constructor
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();

        //check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());


        //constructor with sources
        List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }
        estimator = new NonLinearPositionEstimator3D(sources);

        //check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with fingerprint
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        estimator = new NonLinearPositionEstimator3D(fingerprint);

        //check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with sources and fingerprint
        estimator = new NonLinearPositionEstimator3D(sources, fingerprint);

        //check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(sources,
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with listener
        estimator = new NonLinearPositionEstimator3D(this);

        //check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());


        //constructor with sources and listener
        estimator = new NonLinearPositionEstimator3D(sources, this);

        //check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with fingerprint and listener
        estimator = new NonLinearPositionEstimator3D(fingerprint, this);

        //check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with sources, fingerprint and listener
        estimator = new NonLinearPositionEstimator3D(sources, fingerprint,
                this);

        //check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(sources,
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with initial position
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();
        estimator = new NonLinearPositionEstimator3D(initialPosition);

        //check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());


        //constructor with sources and initial position
        estimator = new NonLinearPositionEstimator3D(sources, initialPosition);

        //check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>)null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with fingerprint and initial position
        estimator = new NonLinearPositionEstimator3D(fingerprint, initialPosition);

        //check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with sources, fingerprint and initial position
        estimator = new NonLinearPositionEstimator3D(sources, fingerprint,
                initialPosition);

        //check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(null, fingerprint,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(sources,
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with initial position and listener
        estimator = new NonLinearPositionEstimator3D(initialPosition,
                this);

        //check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());


        //constructor with sources, initial position and listener
        estimator = new NonLinearPositionEstimator3D(sources, initialPosition,
                this);

        //check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getFingerprint());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    (List<WifiAccessPointLocated3D>)null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with fingerprint, initial position and listener
        estimator = new NonLinearPositionEstimator3D(fingerprint, initialPosition,
                this);

        //check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getSources());
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(
                    (RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>>)null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //constructor with sources, fingerprint, initial position and listener
        estimator = new NonLinearPositionEstimator3D(sources, fingerprint,
                initialPosition, this);

        //check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION, 0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 4);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getPositions());
        assertNull(estimator.getDistances());
        assertNull(estimator.getDistanceStandardDeviations());
        assertNull(estimator.getCovariance());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearPositionEstimator3D(null, fingerprint,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(
                    new ArrayList<WifiAccessPointLocated3D>(), fingerprint,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearPositionEstimator3D(sources,
                    null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();

        //check default value
        assertNull(estimator.getInitialPosition());

        //set new value
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();
        estimator.setInitialPosition(initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testIsSetRadioSourcePositionCovarianceUsed() throws LockedException {
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();

        //check default value
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());

        //set new value
        estimator.setRadioSourcePositionCovarianceUsed(true);

        //check
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testGetSetFallbackDistanceStandardDeviation() throws LockedException {
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();

        //check default value
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);

        //set new value
        estimator.setFallbackDistanceStandardDeviation(5.0);

        //check
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                5.0, 0.0);
    }

    @Test
    public void testGetSetSources() throws LockedException {
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();

        //check default value
        assertNull(estimator.getSources());

        //set new value
        List<WifiAccessPointLocated3D> sources = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sources.add(new WifiAccessPointLocated3D("id1", FREQUENCY,
                    new InhomogeneousPoint3D()));
        }

        estimator.setSources(sources);

        //check
        assertSame(estimator.getSources(), sources);

        //force IllegalArgumentException
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
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();

        //check default value
        assertNull(estimator.getFingerprint());

        //set new value
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        //check
        assertSame(estimator.getFingerprint(), fingerprint);

        //force IllegalArgumentException
        try {
            estimator.setFingerprint(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();

        //check default size
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateNoError() throws LockedException, NotReadyException,
            PositionEstimationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);


            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
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


            NonLinearPositionEstimator3D estimator =
                    new NonLinearPositionEstimator3D(sources, fingerprint, this);

            reset();

            //check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertNotNull(estimator.getDistanceStandardDeviations());
            assertNull(estimator.getCovariance());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            double distance = position.distanceTo(estimatedPosition);
            if (distance < ABSOLUTE_ERROR) {
                assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
                assertNotNull(estimator.getCovariance());
                numValid++;
                break;
            }
        }

        assertTrue(numValid > 0);

        //force NotReadyException
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Test
    public void testEstimateNoErrorWithInitialPosition() throws LockedException,
            NotReadyException, PositionEstimationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);

            InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS),
                    randomizer.nextDouble(MIN_POS, MAX_POS));
            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);


            List<WifiAccessPointWithPowerAndLocated3D> sources = new ArrayList<>();
            List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                InhomogeneousPoint3D accessPointPosition = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS),
                        randomizer.nextDouble(MIN_POS, MAX_POS));

                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                double transmittedPower = Utils.dBmToPower(transmittedPowerdBm);
                String bssid = String.valueOf(i);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(bssid,
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


            NonLinearPositionEstimator3D estimator =
                    new NonLinearPositionEstimator3D(sources, fingerprint, position,
                            this);

            reset();

            //check initial state
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedPositionCoordinates());
            assertNotNull(estimator.getPositions());
            assertNotNull(estimator.getDistances());
            assertNotNull(estimator.getDistanceStandardDeviations());
            assertNull(estimator.getCovariance());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(estimator.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        //force NotReadyException
        NonLinearPositionEstimator3D estimator = new NonLinearPositionEstimator3D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(PositionEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((NonLinearPositionEstimator3D)estimator);
    }

    @Override
    public void onEstimateEnd(PositionEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((NonLinearPositionEstimator3D)estimator);
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

    private void checkLocked(NonLinearPositionEstimator3D estimator) {
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRadioSourcePositionCovarianceUsed(false);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSources(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(null);
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
