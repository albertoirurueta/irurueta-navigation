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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class NonLinearFingerprintPositionAndRadioSourceEstimator2DTest implements
        FingerprintPositionAndRadioSourceEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            NonLinearFingerprintPositionAndRadioSourceEstimator2DTest.class.getName());

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

    private static final double MIN_RSSI_STANDARD_DEVIATION = 1e-2;
    private static final double MAX_RSSI_STANDARD_DEVIATION = 5e-1;

    private static final double MIN_PATHLOSS_STANDARD_DEVIATION = 1e-2;
    private static final double MAX_PATHLOSS_STANDARD_DEVIATION = 5e-2;

    private static final double MIN_POSITION_STANDARD_DEVIATION = 1e-1;
    private static final double MAX_POSITION_STANDARD_DEVIATION = 5e-1;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final double ERROR_STD = 1.0e-1;

    private static final double ERROR_MARGIN = 1e-3;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    public NonLinearFingerprintPositionAndRadioSourceEstimator2DTest() { }

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
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());


        //test constructor with listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());


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

        RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>
                locatedFingerprint = new RssiFingerprintLocated2D<>(readings,
                Point2D.create());

        List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>>
                locatedFingerprints = new ArrayList<>();
        locatedFingerprints.add(locatedFingerprint);

        RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);


        //test constructor with located fingerprints and fingerprint
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                locatedFingerprints, fingerprint);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    locatedFingerprints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                locatedFingerprints, fingerprint, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    locatedFingerprints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and initial position
        Point2D initialPosition = Point2D.create();

        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                locatedFingerprints, fingerprint, initialPosition);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    null, fingerprint, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    locatedFingerprints, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, initial position
        //and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                locatedFingerprints, fingerprint, initialPosition, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertNull(estimator.getInitialLocatedSources());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    null, fingerprint, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    locatedFingerprints, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and
        //initial located sources
        List<RadioSourceLocated<Point2D>> initialLocatedSources = new ArrayList<>();

        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                locatedFingerprints, fingerprint, initialLocatedSources);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertSame(estimator.getInitialLocatedSources(), initialLocatedSources);
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    null, fingerprint, initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    locatedFingerprints, null, initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and initial located sources
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                locatedFingerprints, fingerprint, initialLocatedSources, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertSame(estimator.getInitialLocatedSources(), initialLocatedSources);
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    null, fingerprint, initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    locatedFingerprints, null, initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, initial position and
        //initial located sources
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                locatedFingerprints, fingerprint, initialPosition,
                initialLocatedSources);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertSame(estimator.getInitialLocatedSources(), initialLocatedSources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    null, fingerprint, initialPosition,
                    initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    locatedFingerprints, null, initialPosition,
                    initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, initial position,
        //initial located sources and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                locatedFingerprints, fingerprint, initialPosition,
                initialLocatedSources, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), 1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 2);
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());
        assertSame(estimator.getInitialLocatedSources(), initialLocatedSources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertNull(estimator.getCovariance());
        assertNull(estimator.getEstimatedPositionCovariance());
        assertEquals(estimator.getChiSq(), 0.0, 0.0);
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    null, fingerprint, initialPosition,
                    initialLocatedSources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator2D(
                    locatedFingerprints, null, initialPosition,
                    initialLocatedSources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetLocatedFingerprints() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertNull(estimator.getLocatedFingerprints());

        //set new value
        List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>>
                locatedFingerprints = new ArrayList<>();
        estimator.setLocatedFingerprints(locatedFingerprints);

        //check
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertNull(estimator.getFingerprint());

        //set new value
        RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>();
        estimator.setFingerprint(fingerprint);

        //check
        assertSame(estimator.getFingerprint(), fingerprint);
    }

    @Test
    public void testGetSetMinMaxNearestFingerprints() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

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
            estimator.setMinMaxNearestFingerprints(-1, 3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
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
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

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
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetUseNoMeanNearestFingerprintFinder() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());

        //set new value
        estimator.setUseNoMeanNearestFingerprintFinder(false);

        //check
        assertFalse(estimator.getUseNoMeanNearestFingerprintFinder());
    }

    @Test
    public void testGetSetInitialLocatedSources() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertNull(estimator.getInitialLocatedSources());

        //set new value
        List<RadioSourceLocated<Point2D>> initialLocatedSources = new ArrayList<>();
        estimator.setInitialLocatedSources(initialLocatedSources);

        //check
        assertSame(estimator.getInitialLocatedSources(), initialLocatedSources);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertNull(estimator.getInitialPosition());

        //set new value
        Point2D initialPosition = Point2D.create();
        estimator.setInitialPosition(initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testGetSetUseSourcesPathLossExponentWhenAvailable() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //set new value
        estimator.setUseSourcesPathLossExponentWhenAvailable(false);

        //check
        assertFalse(estimator.getUseSourcesPathLossExponentWhenAvailable());
    }

    @Test
    public void testGetSetFallbackRssiStandardDeviation() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertEquals(estimator.getFallbackRssiStandardDeviation(),
                NonLinearFingerprintPositionAndRadioSourceEstimator.FALLBACK_RSSI_STANDARD_DEVIATION,
                0.0);

        //set new value
        estimator.setFallbackRssiStandardDeviation(0.5);

        //check
        assertEquals(estimator.getFallbackRssiStandardDeviation(), 0.5, 0.0);
    }

    @Test
    public void testIsSetFingerprintRssiStandardDeviationPropagated() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());

        //set new value
        estimator.setFingerprintRssiStandardDeviationPropagated(false);

        //check
        assertFalse(estimator.isFingerprintRssiStandardDeviationPropagated());
    }

    @Test
    public void testIsSetPathlossExponentStandardDeviationPropagated() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());

        //set new value
        estimator.setPathlossExponentStandardDeviationPropagated(false);

        //check
        assertFalse(estimator.isPathlossExponentStandardDeviationPropagated());
    }

    @Test
    public void testIsSetFingerprintPositionCovariancePropagated() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());

        //set new value
        estimator.setFingerprintPositionCovariancePropagated(false);

        //check
        assertFalse(estimator.isFingerprintPositionCovariancePropagated());
    }

    @Test
    public void testIsSetRadioSourcePositionCovariancePropagated() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        //set new value
        estimator.setRadioSourcePositionCovariancePropagated(false);

        //check
        assertFalse(estimator.isRadioSourcePositionCovariancePropagated());
    }


    @Test
    public void testIsSetInitialPositionCovariancePropagated() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator2D();

        //check default value
        assertTrue(estimator.isInitialPositionCovariancePropagated());

        //set new value
        estimator.setInitialPositionCovariancePropagated(false);

        //check
        assertFalse(estimator.isInitialPositionCovariancePropagated());
    }

    //TODO: test estimate

    @Override
    public void onEstimateStart(FingerprintPositionAndRadioSourceEstimator<Point2D> estimator) {
        estimateStart++;
        checkLocked((NonLinearFingerprintPositionAndRadioSourceEstimator2D)estimator);
    }

    @Override
    public void onEstimateEnd(FingerprintPositionAndRadioSourceEstimator<Point2D> estimator) {
        estimateEnd++;
        checkLocked((NonLinearFingerprintPositionAndRadioSourceEstimator2D)estimator);
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

    private void checkLocked(NonLinearFingerprintPositionAndRadioSourceEstimator2D estimator) {
        try {
            estimator.setLocatedFingerprints(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFingerprint(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMinMaxNearestFingerprints(-1 , -1);
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
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialLocatedSources(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setUseSourcesPathLossExponentWhenAvailable(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFallbackRssiStandardDeviation(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFingerprintRssiStandardDeviationPropagated(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPathlossExponentStandardDeviationPropagated(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFingerprintPositionCovariancePropagated(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setRadioSourcePositionCovariancePropagated(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setInitialPositionCovariancePropagated(true);
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
