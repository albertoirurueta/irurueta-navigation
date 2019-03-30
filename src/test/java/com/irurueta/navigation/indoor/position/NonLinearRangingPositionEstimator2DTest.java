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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class NonLinearRangingPositionEstimator2DTest implements
        RangingPositionEstimatorListener<Point2D> {

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    @Test
    public void testConstructor() {
        // empty constructor
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // constructor with sources
        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY,
                    new InhomogeneousPoint2D()));
        }
        estimator = new NonLinearRangingPositionEstimator2D(sources);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprint
        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        estimator = new NonLinearRangingPositionEstimator2D(fingerprint);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources and fingerprint
        estimator = new NonLinearRangingPositionEstimator2D(sources, fingerprint);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(null,
                    fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(sources,
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with listener
        estimator = new NonLinearRangingPositionEstimator2D(this);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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


        // constructor with sources and listener
        estimator = new NonLinearRangingPositionEstimator2D(sources, this);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprint and listener
        estimator = new NonLinearRangingPositionEstimator2D(fingerprint, this);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources, fingerprint and listener
        estimator = new NonLinearRangingPositionEstimator2D(sources, fingerprint,
                this);

        // check default values
        assertNull(estimator.getInitialPosition());
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(sources,
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with initial position
        InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D();
        estimator = new NonLinearRangingPositionEstimator2D(initialPosition);

        // check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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


        // constructor with sources and initial position
        estimator = new NonLinearRangingPositionEstimator2D(sources, initialPosition);

        // check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(),
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprint and initial position
        estimator = new NonLinearRangingPositionEstimator2D(fingerprint,
                initialPosition);

        // check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources, fingerprint and initial position
        estimator = new NonLinearRangingPositionEstimator2D(sources, fingerprint,
                initialPosition);

        // check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(null, fingerprint,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(sources,
                    null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with initial position and listener
        estimator = new NonLinearRangingPositionEstimator2D(initialPosition,
                this);

        // check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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


        // constructor with sources, initial position and listener
        estimator = new NonLinearRangingPositionEstimator2D(sources, initialPosition,
                this);

        // check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    (List<WifiAccessPointLocated2D>)null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(),
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with fingerprint, initial position and listener
        estimator = new NonLinearRangingPositionEstimator2D(fingerprint,
                initialPosition, this);

        // check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    (RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>>)null,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        // constructor with sources, fingerprint, initial position and listener
        estimator = new NonLinearRangingPositionEstimator2D(sources, fingerprint,
                initialPosition, this);

        // check default values
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);
        assertNull(estimator.getEstimatedPosition());
        assertEquals(estimator.getMinRequiredSources(), 3);
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

        // force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearRangingPositionEstimator2D(null, fingerprint,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(
                    new ArrayList<WifiAccessPointLocated2D>(), fingerprint,
                    initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearRangingPositionEstimator2D(sources,
                    null, initialPosition, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();

        // check default value
        assertNull(estimator.getInitialPosition());

        // set new value
        InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D();
        estimator.setInitialPosition(initialPosition);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testIsSetRadioSourcePositionCovarianceUsed() throws LockedException {
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();

        // check default value
        assertFalse(estimator.isRadioSourcePositionCovarianceUsed());

        // set new value
        estimator.setRadioSourcePositionCovarianceUsed(true);

        // check
        assertTrue(estimator.isRadioSourcePositionCovarianceUsed());
    }

    @Test
    public void testGetSetFallbackDistanceStandardDeviation() throws LockedException {
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();

        // check default value
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                NonLinearRangingPositionEstimator.FALLBACK_DISTANCE_STANDARD_DEVIATION,
                0.0);

        // set new value
        estimator.setFallbackDistanceStandardDeviation(5.0);

        // check
        assertEquals(estimator.getFallbackDistanceStandardDeviation(),
                5.0, 0.0);
    }

    @Test
    public void testGetSetSources() throws LockedException {
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();

        // check default value
        assertNull(estimator.getSources());

        // set new value
        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY,
                    new InhomogeneousPoint2D()));
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
            estimator.setSources(new ArrayList<WifiAccessPointLocated2D>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();

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
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();

        // check default size
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateNoError() throws LockedException, NotReadyException,
            PositionEstimationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
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


            NonLinearRangingPositionEstimator2D estimator =
                    new NonLinearRangingPositionEstimator2D(sources, fingerprint,
                            this);

            reset();

            // check initial state
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

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            if (!position.equals(estimatedPosition, 10.0 * ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, 10.0 * ABSOLUTE_ERROR));
            assertNotNull(estimator.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();
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


            NonLinearRangingPositionEstimator2D estimator =
                    new NonLinearRangingPositionEstimator2D(sources, fingerprint,
                            position, this);

            reset();

            // check initial state
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

            Point2D estimatedPosition = estimator.getEstimatedPosition();
            if (estimatedPosition.distanceTo(position) > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(position.equals(estimatedPosition, ABSOLUTE_ERROR));
            assertNotNull(estimator.getCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // force NotReadyException
        NonLinearRangingPositionEstimator2D estimator =
                new NonLinearRangingPositionEstimator2D();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) { }
    }

    @Override
    public void onEstimateStart(RangingPositionEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((NonLinearRangingPositionEstimator2D) estimator);
    }

    @Override
    public void onEstimateEnd(RangingPositionEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((NonLinearRangingPositionEstimator2D) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void checkLocked(NonLinearRangingPositionEstimator2D estimator) {
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
